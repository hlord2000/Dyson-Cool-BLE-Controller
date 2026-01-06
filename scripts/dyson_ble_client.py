#!/usr/bin/env python3
"""
Dyson Cool BLE Controller Client

A production-quality BLE client for controlling the Dyson Cool fan controller
using the bleak library. Supports daemon mode, auto-reconnect with exponential
backoff, and comprehensive CLI interface.

Usage:
    # Scan for available devices
    python dyson_ble_client.py scan

    # Send a command to the fan
    python dyson_ble_client.py command power
    python dyson_ble_client.py command speed_up --device "Dyson Cool"

    # Run in daemon mode (background service)
    python dyson_ble_client.py daemon --device "Dyson Cool"

Author: Generated for Dyson Cool BLE Controller
License: MIT
"""

from __future__ import annotations

import argparse
import asyncio
import enum
import logging
import os
import signal
import sys
from dataclasses import dataclass
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from bleak.exc import BleakError

# =============================================================================
# Constants
# =============================================================================

DEFAULT_DEVICE_NAME = "Dyson Cool"
GATT_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
FAN_CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"

# Reconnection settings
INITIAL_RECONNECT_DELAY = 1.0  # seconds
MAX_RECONNECT_DELAY = 60.0  # seconds
RECONNECT_BACKOFF_FACTOR = 2.0

# Write retry settings
MAX_WRITE_RETRIES = 3
WRITE_RETRY_DELAY = 0.5  # seconds

# Scan settings
SCAN_TIMEOUT = 10.0  # seconds


# =============================================================================
# Fan Commands
# =============================================================================


class FanCommand(enum.Enum):
    """Enumeration of available fan control commands."""

    POWER = 0x01
    OSCILLATE = 0x02
    SPEED_UP = 0x03
    SPEED_DOWN = 0x04
    TIMER_UP = 0x05
    TIMER_DOWN = 0x06

    @classmethod
    def from_string(cls, name: str) -> "FanCommand":
        """
        Convert a string command name to FanCommand enum.

        Args:
            name: Command name (case-insensitive, underscores optional)

        Returns:
            Corresponding FanCommand enum value

        Raises:
            ValueError: If command name is not recognized
        """
        normalized = name.upper().replace("-", "_")
        try:
            return cls[normalized]
        except KeyError:
            valid_commands = ", ".join(cmd.name.lower() for cmd in cls)
            raise ValueError(
                f"Unknown command: {name}. Valid commands: {valid_commands}"
            )


# =============================================================================
# Connection State
# =============================================================================


class ConnectionState(enum.Enum):
    """Enumeration of possible connection states."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"


@dataclass
class ConnectionStatus:
    """Tracks the current connection status and statistics."""

    state: ConnectionState = ConnectionState.DISCONNECTED
    device_address: Optional[str] = None
    device_name: Optional[str] = None
    connection_attempts: int = 0
    successful_writes: int = 0
    failed_writes: int = 0
    last_error: Optional[str] = None

    def reset_stats(self) -> None:
        """Reset connection statistics."""
        self.connection_attempts = 0
        self.successful_writes = 0
        self.failed_writes = 0
        self.last_error = None


# =============================================================================
# Dyson BLE Client
# =============================================================================


class DysonBLEClient:
    """
    Async BLE client for controlling Dyson Cool fan.

    This client provides:
    - Automatic device discovery by name
    - Auto-reconnect with exponential backoff
    - Retry logic for failed writes
    - Connection state tracking
    - Daemon mode for continuous operation

    Attributes:
        device_name: Name of the BLE device to connect to
        status: Current connection status and statistics
        logger: Logger instance for this client
    """

    def __init__(
        self,
        device_name: str = DEFAULT_DEVICE_NAME,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """
        Initialize the Dyson BLE Client.

        Args:
            device_name: Name of the BLE device to connect to
            logger: Optional logger instance (creates one if not provided)
        """
        self.device_name = device_name
        self.logger = logger or logging.getLogger(__name__)
        self.status = ConnectionStatus()

        self._client: Optional[BleakClient] = None
        self._device: Optional[BLEDevice] = None
        self._shutdown_event = asyncio.Event()
        self._reconnect_task: Optional[asyncio.Task] = None
        self._current_reconnect_delay = INITIAL_RECONNECT_DELAY

    @property
    def is_connected(self) -> bool:
        """Check if client is currently connected."""
        return (
            self._client is not None
            and self._client.is_connected
            and self.status.state == ConnectionState.CONNECTED
        )

    async def scan_devices(
        self, timeout: float = SCAN_TIMEOUT
    ) -> list[tuple[BLEDevice, AdvertisementData]]:
        """
        Scan for available BLE devices.

        Args:
            timeout: Scan timeout in seconds

        Returns:
            List of tuples containing (device, advertisement_data)
        """
        self.logger.info(f"Scanning for BLE devices (timeout: {timeout}s)...")
        devices = await BleakScanner.discover(
            timeout=timeout, return_adv=True
        )
        self.logger.info(f"Found {len(devices)} devices")
        return list(devices.values())

    async def find_device(
        self, timeout: float = SCAN_TIMEOUT
    ) -> Optional[BLEDevice]:
        """
        Find the target device by name.

        Args:
            timeout: Scan timeout in seconds

        Returns:
            BLEDevice if found, None otherwise
        """
        self.logger.info(f"Searching for device: {self.device_name}")

        def match_device(
            device: BLEDevice, adv_data: AdvertisementData
        ) -> bool:
            return device.name == self.device_name

        device = await BleakScanner.find_device_by_filter(
            match_device, timeout=timeout
        )

        if device:
            self.logger.info(
                f"Found device: {device.name} ({device.address})"
            )
        else:
            self.logger.warning(
                f"Device '{self.device_name}' not found after {timeout}s scan"
            )

        return device

    def _on_disconnect(self, client: BleakClient) -> None:
        """
        Handle disconnection event.

        Args:
            client: The disconnected BleakClient
        """
        self.logger.warning(f"Disconnected from {self.status.device_address}")
        self.status.state = ConnectionState.DISCONNECTED

    async def connect(self, timeout: float = SCAN_TIMEOUT) -> bool:
        """
        Connect to the target device.

        Args:
            timeout: Timeout for device discovery and connection

        Returns:
            True if connection successful, False otherwise
        """
        self.status.state = ConnectionState.CONNECTING
        self.status.connection_attempts += 1

        try:
            # Find the device if we don't have it cached
            if self._device is None:
                self._device = await self.find_device(timeout=timeout)
                if self._device is None:
                    self.status.state = ConnectionState.DISCONNECTED
                    self.status.last_error = "Device not found"
                    return False

            self.status.device_address = self._device.address
            self.status.device_name = self._device.name

            # Create and connect the client
            self._client = BleakClient(
                self._device,
                disconnected_callback=self._on_disconnect,
            )

            self.logger.info(
                f"Connecting to {self._device.name} ({self._device.address})..."
            )
            await self._client.connect()

            if self._client.is_connected:
                self.status.state = ConnectionState.CONNECTED
                self._current_reconnect_delay = INITIAL_RECONNECT_DELAY
                self.logger.info("Connected successfully")

                # Log available services for debugging
                if self.logger.isEnabledFor(logging.DEBUG):
                    for service in self._client.services:
                        self.logger.debug(f"Service: {service.uuid}")
                        for char in service.characteristics:
                            self.logger.debug(
                                f"  Characteristic: {char.uuid} "
                                f"(properties: {char.properties})"
                            )

                return True
            else:
                self.status.state = ConnectionState.DISCONNECTED
                self.status.last_error = "Connection failed"
                return False

        except BleakError as e:
            self.logger.error(f"BLE connection error: {e}")
            self.status.state = ConnectionState.DISCONNECTED
            self.status.last_error = str(e)
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error during connection: {e}")
            self.status.state = ConnectionState.DISCONNECTED
            self.status.last_error = str(e)
            return False

    async def disconnect(self) -> None:
        """Disconnect from the device and clean up resources."""
        if self._reconnect_task and not self._reconnect_task.done():
            self._reconnect_task.cancel()
            try:
                await self._reconnect_task
            except asyncio.CancelledError:
                pass
            self._reconnect_task = None

        if self._client and self._client.is_connected:
            self.logger.info("Disconnecting from device...")
            try:
                await self._client.disconnect()
            except BleakError as e:
                self.logger.warning(f"Error during disconnect: {e}")

        self._client = None
        self.status.state = ConnectionState.DISCONNECTED
        self.logger.info("Disconnected")

    async def _reconnect_loop(self) -> None:
        """
        Internal reconnection loop with exponential backoff.

        This method runs continuously, attempting to reconnect when
        disconnected until the shutdown event is set.
        """
        while not self._shutdown_event.is_set():
            if self.status.state == ConnectionState.DISCONNECTED:
                self.status.state = ConnectionState.RECONNECTING
                self.logger.info(
                    f"Attempting reconnection in {self._current_reconnect_delay:.1f}s..."
                )

                try:
                    await asyncio.wait_for(
                        self._shutdown_event.wait(),
                        timeout=self._current_reconnect_delay,
                    )
                    break  # Shutdown requested
                except asyncio.TimeoutError:
                    pass  # Continue with reconnection

                # Clear cached device to force re-scan
                self._device = None

                if await self.connect():
                    self.logger.info("Reconnection successful")
                else:
                    # Increase backoff delay
                    self._current_reconnect_delay = min(
                        self._current_reconnect_delay * RECONNECT_BACKOFF_FACTOR,
                        MAX_RECONNECT_DELAY,
                    )
                    self.logger.warning(
                        f"Reconnection failed, next attempt in "
                        f"{self._current_reconnect_delay:.1f}s"
                    )
            else:
                # Wait a bit before checking connection state again
                try:
                    await asyncio.wait_for(
                        self._shutdown_event.wait(), timeout=1.0
                    )
                    break
                except asyncio.TimeoutError:
                    pass

    async def send_command(
        self,
        command: FanCommand,
        retries: int = MAX_WRITE_RETRIES,
    ) -> bool:
        """
        Send a command to the fan.

        Args:
            command: The FanCommand to send
            retries: Number of retry attempts on failure

        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.is_connected:
            self.logger.error("Not connected to device")
            return False

        data = bytes([command.value])

        for attempt in range(retries):
            try:
                self.logger.info(
                    f"Sending command: {command.name} (0x{command.value:02X})"
                )
                await self._client.write_gatt_char(
                    FAN_CONTROL_CHAR_UUID, data, response=True
                )
                self.status.successful_writes += 1
                self.logger.info(f"Command {command.name} sent successfully")
                return True

            except BleakError as e:
                self.logger.warning(
                    f"Write attempt {attempt + 1}/{retries} failed: {e}"
                )
                self.status.failed_writes += 1
                self.status.last_error = str(e)

                if attempt < retries - 1:
                    await asyncio.sleep(WRITE_RETRY_DELAY)

            except Exception as e:
                self.logger.error(f"Unexpected error during write: {e}")
                self.status.failed_writes += 1
                self.status.last_error = str(e)
                return False

        self.logger.error(
            f"Failed to send command {command.name} after {retries} attempts"
        )
        return False

    async def run_daemon(
        self,
        command_callback: Optional[Callable[[str], asyncio.Future]] = None,
    ) -> None:
        """
        Run in daemon mode with auto-reconnect.

        This method runs until shutdown is signaled via signal handlers
        or by calling shutdown().

        Args:
            command_callback: Optional async callback for handling external
                              commands (e.g., from stdin or IPC)
        """
        self.logger.info("Starting daemon mode...")
        self._shutdown_event.clear()

        # Initial connection
        if not await self.connect():
            self.logger.warning(
                "Initial connection failed, will retry in background"
            )

        # Start reconnection loop
        self._reconnect_task = asyncio.create_task(self._reconnect_loop())

        try:
            # Wait for shutdown signal
            await self._shutdown_event.wait()
        except asyncio.CancelledError:
            self.logger.info("Daemon cancelled")
        finally:
            await self.disconnect()
            self.logger.info("Daemon stopped")

    def shutdown(self) -> None:
        """Signal the daemon to shut down gracefully."""
        self.logger.info("Shutdown requested")
        self._shutdown_event.set()

    def get_status_report(self) -> str:
        """
        Get a human-readable status report.

        Returns:
            Formatted status string
        """
        return (
            f"Connection Status Report\n"
            f"========================\n"
            f"State: {self.status.state.value}\n"
            f"Device: {self.status.device_name or 'N/A'}\n"
            f"Address: {self.status.device_address or 'N/A'}\n"
            f"Connection attempts: {self.status.connection_attempts}\n"
            f"Successful writes: {self.status.successful_writes}\n"
            f"Failed writes: {self.status.failed_writes}\n"
            f"Last error: {self.status.last_error or 'None'}\n"
        )


# =============================================================================
# CLI Functions
# =============================================================================


def setup_logging(verbose: bool = False, debug: bool = False) -> logging.Logger:
    """
    Configure logging for the application.

    Args:
        verbose: Enable verbose output (INFO level)
        debug: Enable debug output (DEBUG level, implies verbose)

    Returns:
        Configured logger instance
    """
    if debug:
        level = logging.DEBUG
    elif verbose:
        level = logging.INFO
    else:
        level = logging.WARNING

    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    return logging.getLogger("dyson_ble_client")


async def cmd_scan(args: argparse.Namespace, logger: logging.Logger) -> int:
    """
    Handle the 'scan' command.

    Args:
        args: Parsed command-line arguments
        logger: Logger instance

    Returns:
        Exit code (0 for success)
    """
    client = DysonBLEClient(logger=logger)
    devices = await client.scan_devices(timeout=args.timeout)

    if not devices:
        print("No BLE devices found")
        return 0

    print(f"\nFound {len(devices)} BLE device(s):\n")
    print(f"{'Name':<30} {'Address':<20} {'RSSI':<8}")
    print("-" * 60)

    for device, adv_data in sorted(
        devices, key=lambda x: x[1].rssi or -999, reverse=True
    ):
        name = device.name or "(Unknown)"
        rssi = adv_data.rssi if adv_data.rssi is not None else "N/A"
        print(f"{name:<30} {device.address:<20} {rssi:<8}")

    return 0


async def cmd_command(args: argparse.Namespace, logger: logging.Logger) -> int:
    """
    Handle the 'command' subcommand.

    Args:
        args: Parsed command-line arguments
        logger: Logger instance

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    try:
        command = FanCommand.from_string(args.cmd)
    except ValueError as e:
        logger.error(str(e))
        return 1

    client = DysonBLEClient(device_name=args.device, logger=logger)

    try:
        if not await client.connect(timeout=args.timeout):
            logger.error("Failed to connect to device")
            return 1

        if await client.send_command(command):
            print(f"Command '{command.name}' sent successfully")
            return 0
        else:
            logger.error(f"Failed to send command '{command.name}'")
            return 1

    finally:
        await client.disconnect()


async def cmd_daemon(args: argparse.Namespace, logger: logging.Logger) -> int:
    """
    Handle the 'daemon' subcommand.

    Args:
        args: Parsed command-line arguments
        logger: Logger instance

    Returns:
        Exit code (0 for success)
    """
    client = DysonBLEClient(device_name=args.device, logger=logger)

    # Set up signal handlers
    loop = asyncio.get_running_loop()

    def signal_handler(sig: signal.Signals) -> None:
        logger.info(f"Received signal {sig.name}")
        client.shutdown()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler, sig)

    print(f"Starting daemon mode for device: {args.device}")
    print("Press Ctrl+C to stop")

    await client.run_daemon()

    print("\n" + client.get_status_report())
    return 0


def create_parser() -> argparse.ArgumentParser:
    """
    Create the command-line argument parser.

    Returns:
        Configured ArgumentParser instance
    """
    parser = argparse.ArgumentParser(
        prog="dyson_ble_client",
        description="BLE client for controlling Dyson Cool fan",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s scan                           Scan for BLE devices
  %(prog)s command power                  Toggle power on/off
  %(prog)s command speed_up -d "My Fan"   Increase speed on custom device
  %(prog)s daemon -v                      Run in daemon mode with verbose output

Available commands:
  power      - Toggle power on/off (0x01)
  oscillate  - Toggle oscillation (0x02)
  speed_up   - Increase fan speed (0x03)
  speed_down - Decrease fan speed (0x04)
  timer_up   - Increase timer (0x05)
  timer_down - Decrease timer (0x06)
        """,
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output (implies --verbose)",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=SCAN_TIMEOUT,
        help=f"Scan/connection timeout in seconds (default: {SCAN_TIMEOUT})",
    )

    subparsers = parser.add_subparsers(
        title="commands",
        dest="subcommand",
        required=True,
        help="Available commands",
    )

    # Scan subcommand
    scan_parser = subparsers.add_parser(
        "scan",
        help="Scan for available BLE devices",
    )
    scan_parser.set_defaults(func=cmd_scan)

    # Command subcommand
    cmd_parser = subparsers.add_parser(
        "command",
        help="Send a command to the fan",
    )
    cmd_parser.add_argument(
        "cmd",
        type=str,
        help="Command to send (power, oscillate, speed_up, speed_down, "
        "timer_up, timer_down)",
    )
    cmd_parser.add_argument(
        "-d",
        "--device",
        type=str,
        default=DEFAULT_DEVICE_NAME,
        help=f"Device name to connect to (default: {DEFAULT_DEVICE_NAME})",
    )
    cmd_parser.set_defaults(func=cmd_command)

    # Daemon subcommand
    daemon_parser = subparsers.add_parser(
        "daemon",
        help="Run in daemon mode with auto-reconnect",
    )
    daemon_parser.add_argument(
        "-d",
        "--device",
        type=str,
        default=DEFAULT_DEVICE_NAME,
        help=f"Device name to connect to (default: {DEFAULT_DEVICE_NAME})",
    )
    daemon_parser.set_defaults(func=cmd_daemon)

    return parser


async def main() -> int:
    """
    Main entry point for the CLI.

    Returns:
        Exit code
    """
    parser = create_parser()
    args = parser.parse_args()

    logger = setup_logging(verbose=args.verbose, debug=args.debug)

    try:
        return await args.func(args, logger)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 130
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        if args.debug:
            import traceback

            traceback.print_exc()
        return 1


if __name__ == "__main__":
    # Handle running on Windows where ProactorEventLoop doesn't support
    # add_signal_handler
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    sys.exit(asyncio.run(main()))
