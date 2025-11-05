#!/usr/bin/env python3
import spidev
import RPi.GPIO as GPIO
import time
import sys

# --- SPI and GPIO Setup ---
SPI_BUS = 1
SPI_DEVICE = 0
GPIO_RESET = 17    # GPIO pin for Reset
GPIO_CS = 18        # GPIO pin for Chip Select (connected to NSS)
GPIO_DIO0 = 24     # GPIO pin for DIO0 (interrupt, mapped to TxDone or PayloadReady)

# --- Register Definitions for FSK/OOK Mode ---
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_BITRATE_MSB = 0x02
REG_BITRATE_LSB = 0x03
REG_FDEV_MSB = 0x04
REG_FDEV_LSB = 0x05
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_VERSION = 0x42

# Power Amplifier (PA) Configuration
REG_PA_CONFIG = 0x09
REG_PA_RAMP = 0x0A

# FSK Packet and FIFO Configuration
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F # Added for good practice
REG_PREAMBLE_MSB = 0x25
REG_PREAMBLE_LSB = 0x26
REG_SYNC_CONFIG = 0x27
REG_SYNC_VALUE_1 = 0x28
REG_PACKET_CONFIG_1 = 0x30    # Packet Config 1 (FSK)
REG_PAYLOAD_LENGTH_FSK = 0x32 # Payload Length (FSK)
REG_IRQ_FLAGS_1 = 0x3E
REG_IRQ_FLAGS_2 = 0x3F
REG_DIO_MAPPING_1 = 0x40

# --- FSK/OOK Mode Settings ---
FREQ = 868000000       # 868 MHz
BITRATE = 9600         # 9600 bps
FDEV = 5000            # FSK deviation in Hz (5 kHz)
PREAMBLE_SIZE_SYMBOLS = 8 # 8 bytes
SYNC_WORD = [0xE1, 0x5A, 0xE8, 0x93] # 4-byte Sync Word


# ---------------------------------------------------------------------
# SPI Helper Functions
# ---------------------------------------------------------------------
def spi_write_register(reg, value):
    """Write a single byte to an SX1276 register."""
    spi.xfer2([reg | 0x80, value])

def spi_read_register(reg):
    """Read a single byte from an SX1276 register."""
    resp = spi.xfer2([reg & 0x7F, 0x00])
    return resp[1]

def calculate_frf(freq):
    """Calculate Frequency Register Value (24-bit). Fstep = 61.03515625 Hz."""
    frf = int(freq / 61.03515625)
    return (frf >> 16) & 0xFF, (frf >> 8) & 0xFF, frf & 0xFF

def calculate_bitrate(bitrate):
    """Calculate Bitrate Register Value (16-bit). Rbitrate = Fosc / Bitrate."""
    br_val = int(32000000 / bitrate)
    return (br_val >> 8) & 0xFF, br_val & 0xFF

def calculate_fdev(fdev):
    """Calculate Frequency Deviation Register Value (16-bit). Fdev = Fstep * RegFdev."""
    fdev_val = int(fdev / 61.03515625)
    return (fdev_val >> 8) & 0xFF, fdev_val & 0xFF


# ---------------------------------------------------------------------
# Module Ping / Connectivity Check
# ---------------------------------------------------------------------
def ping_module():
    try:
        version = spi_read_register(REG_VERSION)
        if version == 0x12:
            print(f"Ping successful! SX1276 Version Register (0x42) reads: {hex(version)}")
            return True
        else:
            print(f"Failed to ping! Version Register (0x42) returned: {hex(version)} (Expected: 0x12)")
            return False
    except Exception as e:
        print(f"Ping failed due to SPI error: {e}")
        return False


# ---------------------------------------------------------------------
# SX1276 Setup (FSK Mode)
# ---------------------------------------------------------------------
def setup_sx1276():
    """Configures the SX1276 module for FSK transmission."""
    
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_RESET, GPIO.OUT)
    GPIO.setup(GPIO_CS, GPIO.OUT)
    GPIO.setup(GPIO_DIO0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
    
    # Reset the module
    GPIO.output(GPIO_RESET, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(GPIO_RESET, GPIO.HIGH)
    time.sleep(0.01)
    
    if not ping_module():
        print("SX1276 module not responding correctly. Aborting.")
        sys.exit(1)
            
    # 1. Enter Sleep mode and set FSK/OOK mode
    spi_write_register(REG_OP_MODE, 0x00)  # 0x00 = FSK/OOK mode, Sleep
    time.sleep(0.01)
    
    # 2. Enter Standby mode
    spi_write_register(REG_OP_MODE, 0x02)  # 0x02 = FSK/OOK mode, Standby
    time.sleep(0.01)
    
    # 3. Set frequency
    frf_msb, frf_mid, frf_lsb = calculate_frf(FREQ)
    spi_write_register(REG_FRF_MSB, frf_msb)
    spi_write_register(REG_FRF_MID, frf_mid)
    spi_write_register(REG_FRF_LSB, frf_lsb)
    
    # 4. Set bitrate
    br_msb, br_lsb = calculate_bitrate(BITRATE)
    spi_write_register(REG_BITRATE_MSB, br_msb)
    spi_write_register(REG_BITRATE_LSB, br_lsb)
    
    # 5. Set FSK deviation
    fdev_msb, fdev_lsb = calculate_fdev(FDEV)
    spi_write_register(REG_FDEV_MSB, fdev_msb)
    spi_write_register(REG_FDEV_LSB, fdev_lsb)
    
    # 6. Set Preamble Length (in bytes)
    spi_write_register(REG_PREAMBLE_MSB, 0x00)
    spi_write_register(REG_PREAMBLE_LSB, PREAMBLE_SIZE_SYMBOLS)
    
    # 7. Configure Sync Word (Access Code)
    spi_write_register(REG_SYNC_CONFIG, 0x83)  # SyncOn=1, SyncSize=4 bytes
    for i, byte in enumerate(SYNC_WORD):
        spi_write_register(REG_SYNC_VALUE_1 + i, byte)
        
    # 8. Configure Packet Format (Variable length)
    spi_write_register(REG_PACKET_CONFIG_1, 0x80)
    
    # 9. Set FIFO TX/RX Base Addresses
    spi_write_register(REG_FIFO_TX_BASE_ADDR, 0x00)
    spi_write_register(REG_FIFO_RX_BASE_ADDR, 0x00)
    
    # 10. Configure PA Output Power (PA_BOOST)
    spi_write_register(REG_PA_CONFIG, 0x8F)
    
    # 11. Set PA Ramp Time
    spi_write_register(REG_PA_RAMP, 0x09)
    
    # 12. Map DIO0 to TxDone (for TX) or PayloadReady (for RX)
    spi_write_register(REG_DIO_MAPPING_1, 0x00)
    
    print("SX1276 FSK setup complete.")


# ---------------------------------------------------------------------
# Transmission Function
# ---------------------------------------------------------------------
def radio_transmit_data(data):
    """Transmit data in FSK mode."""
    
    payload = list(data)
    payload_len = len(payload)
    
    if payload_len > 64:
        print(f"Error: Payload too long ({payload_len} bytes). FSK FIFO is 64 bytes.")
        return
    
    spi_write_register(REG_OP_MODE, 0x02)  # Standby
    time.sleep(0.005)
    spi_read_register(REG_IRQ_FLAGS_1)  # Clear IRQ
    
    spi_write_register(REG_FIFO_ADDR_PTR, 0x00)
    
    fifo_data = [payload_len] + payload
    print(f"Writing {len(fifo_data)} bytes to FIFO (Len: {payload_len}, Payload: '{data.decode()[:20]}...')")
    
    spi.xfer2([REG_FIFO | 0x80] + fifo_data)
    
    spi_write_register(REG_OP_MODE, 0x03)  # Transmit mode
    print("Starting transmission...")
    
    start_time = time.time()
    total_bytes = PREAMBLE_SIZE_SYMBOLS + len(SYNC_WORD) + 1 + payload_len
    time_on_air = (total_bytes * 8) / BITRATE
    timeout = max(1.0, time_on_air * 5.0)
    tx_done = False

    while time.time() - start_time < timeout:
        irq_flags = spi_read_register(REG_IRQ_FLAGS_1)
        if irq_flags & 0x08:  # TxDone flag
            tx_done = True
            break
        time.sleep(0.01)
        
    spi_write_register(REG_OP_MODE, 0x02)
    
    if tx_done:
        spi_read_register(REG_IRQ_FLAGS_1)
        print("Transmission complete (TxDone confirmed).")
    else:
        print(f"Transmission failed: Timeout after {timeout:.2f} s.")


# ---------------------------------------------------------------------
# Reception Function (NEW)
# ---------------------------------------------------------------------
def radio_receive_data(timeout=5.0):
    """
    Receive a packet in FSK mode.
    Returns the received data as bytes, or None on timeout.
    """
    
    spi_write_register(REG_OP_MODE, 0x02)  # Standby
    time.sleep(0.005)
    
    spi_read_register(REG_IRQ_FLAGS_1)
    spi_read_register(REG_IRQ_FLAGS_2)
    
    spi_write_register(REG_FIFO_ADDR_PTR, 0x00)
    spi_write_register(REG_DIO_MAPPING_1, 0x00)  # DIO0 = PayloadReady
    
    spi_write_register(REG_OP_MODE, 0x10)  # Receive mode
    print("Radio set to RX mode...")
    
    start_time = time.time()
    payload_ready = False
    
    while time.time() - start_time < timeout:
        irq_flags2 = spi_read_register(REG_IRQ_FLAGS_2)
        if irq_flags2 & 0x40:  # PayloadReady (bit 6)
            payload_ready = True
            break
        time.sleep(0.01)
    
    if not payload_ready:
        spi_write_register(REG_OP_MODE, 0x02)
        print(f"No packet received within {timeout}s.")
        return None

    spi_write_register(REG_FIFO_ADDR_PTR, 0x00)
    length = spi.xfer2([REG_FIFO & 0x7F, 0x00])[1]
    
    if length == 0 or length > 64:
        print(f"Invalid packet length: {length}")
        return None
    
    rx_bytes = []
    for _ in range(length):
        rx_bytes.append(spi.xfer2([REG_FIFO & 0x7F, 0x00])[1])
    
    spi_write_register(REG_OP_MODE, 0x02)
    spi_read_register(REG_IRQ_FLAGS_1)
    spi_read_register(REG_IRQ_FLAGS_2)
    
    payload = bytes(rx_bytes)
    try:
        decoded = payload.decode('utf-8')
        print(f"Packet received ({length} bytes): '{decoded}'")
    except UnicodeDecodeError:
        print(f"Binary packet received ({length} bytes): {payload}")
    
    return payload


# ---------------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------------
if __name__ == "__main__":
    spi = spidev.SpiDev()
    try:
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = 1000000
        
        setup_sx1276()
        
        mode = input("Select mode (T = Transmit, R = Receive): ").strip().upper()
        
        if mode == 'T':
            counter = 0
            print("\n--- TRANSMIT MODE ---")
            while True:
                counter += 1
                message = f"Packet {counter}: SX1276 FSK Test."
                radio_transmit_data(message.encode('utf-8'))
                time.sleep(2)
        
        elif mode == 'R':
            print("\n--- RECEIVE MODE ---")
            while True:
                radio_receive_data(timeout=10)
                time.sleep(0.5)
        
        else:
            print("Invalid mode selected.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'spi' in locals() and spi:
            spi.close()
        GPIO.cleanup()
        print("Cleanup complete.")
