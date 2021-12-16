'''
LiDAR SWARM
LoRAWan through the SWARM Space Constillation
J. King 2021
'''
import time
import rtc
import asyncio
import adafruit_logging as logging
import random
from adafruit_datetime import datetime
import board
import digitalio
import busio
import adafruit_rfm9x
import terminalio
import neopixel
from barbudor_ina3221 import *
from microcontroller import watchdog as w
from watchdog import WatchDogMode

# Setup logging
logger = logging.getLogger("lidarSwarm")
logger.setLevel(logging.DEBUG)
logger.info("LiDAR SWARM BOOT")

# INIT WATCHDOG
w.timeout = 600
w.mode = WatchDogMode.RESET
w.feed() # Good doggo

# Init the neopixels
pixels = neopixel.NeoPixel(board.IO38, 2, bpp=4, pixel_order=neopixel.GRBW)

# Init SPI for the LoRa radio
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D6)
reset = digitalio.DigitalInOut(board.D9)

# Init I2C for the INA
i2c = board.I2C()

# Define a class for the Lora modem and data packets
class LORA():
    def __init__(self, spi, reset, cs):
        self._nodeFreq = 915.4
        self.connected = False
        self.ts = []
        self.msg = []
        try:
            self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, self._nodeFreq)
            self.connected = True
            logger.info("LoRa Connection OK")
        except Exception as e:
            logger.info("LoRa Connection Error")
    def add_packet(self, ts, msg):
        self.ts.append(ts)
        self.msg.append(msg)


# Class for INA3221 voltage and current sensing
class INA():
    def __init__(self, i2c):
        try:
            self.chans = ['BAT','SOL','3V3']
            self.sensor = INA3221(i2c, shunt_resistor = (0.01, 0.01, 0.01))
            self.sensor.update(reg=C_REG_CONFIG, mask=C_AVERAGING_MASK | C_VBUS_CONV_TIME_MASK | C_SHUNT_CONV_TIME_MASK | C_MODE_MASK,
                                value=C_AVERAGING_128_SAMPLES | C_VBUS_CONV_TIME_8MS | C_SHUNT_CONV_TIME_8MS | C_MODE_SHUNT_AND_BUS_CONTINOUS)
            self.sensor.enable_channel(1)
            self.sensor.enable_channel(2)
            self.sensor.enable_channel(3)
            self.connected = True
            logger.info("INA Connection OK")
        except:
            self.connected = False
            logger.error("INA Connection Error")

        self.bus_v = [None, None, None]
        self.bus_c = [None, None, None]
        self.last = None

    def update(self):
        try:
            for c_idx, c_name in enumerate(self.chans):
                self.bus_v[c_idx] = self.sensor.bus_voltage(c_idx+1)
                self.bus_c[c_idx] = self.sensor.current(c_idx+1)
                self.last = time.time()
                self.connected = False
                logger.debug('%s: %.2fv,  %.2fA' % (c_name,self.bus_v[c_idx], self.bus_c[c_idx]))
        except Exception as e:
            logger.error("INA Connection Error")
            self.connected = False

# Define class for Swarm Tile
class TILE():
    def __init__(self):
        self.lat = None # in d.dddd (float)
        self.lon = None # in d.dddd (float)
        self.altitude = None # in m (float)
        self.course = None # in deg (float)
        self.speed = None # in km/h (float)
        self.gpsts = None  # in YYYYMMDDhhmmss
        self.validDate = False # Flag for fresh time update
        self.validFix = False # Flag for fresh GPS update
        self.connected = False # Flag for tile state (UART reponding)
        self.asleep = False # Flag for sleep status of the tile
        self.errorCount = 0 # Counter for errors encountered

        # See for Swarm Tile Product Manual for command details
        self.cmd ={'getFirm': '$FV',
                   'getDate': '$DT @',
                   'getGPS': '$GN @',
                   'getFix': '$GS @',
                   'reboot': '$RS',
                   'sleep': '$SL',
                   'transmit': '$TD'}

        # Establish UART connection with Tile
        self.uart = busio.UART(board.TX,board.RX,
                                   baudrate=115200,
                                   receiver_buffer_size=8192,
                                   timeout=0.0)

    # Queue data to transmit vis SWARM
    # expiry sets max time before a message is deleted
    def transmit(self, data, expiry=7200):
        if expiry is not None:
            cmd = self.cmd['transmit'] +' HD='+str(expiry)+',"'+data+'"'
        self.write(cmd)
        header, msg = self.read(timeout = 5,
                                returnData = True,
                                headerType = self.cmd['transmit'])
        if msg[0] == 'OK':
            logger.info('Tile Transmit OK')
        else:
            logger.error('Tile Transmit Error')
            self.errorCount +=1

    # Does a full cold reboot of the tile
    def reboot(self):
        self.write(self.cmd['reboot'])
        logger.info('Tile Cold Reboot')

    # Put the tile to sleep
    def sleep(self, seconds=3600):
        cmd = self.cmd['sleep'] +' S='+str(seconds)
        self.write(cmd)
        header, msg = self.read(timeout = 5,
                                returnData = True,
                                headerType = self.cmd['sleep'])
        if msg[0] == 'OK':
            logger.info('Tile Sleeping OK')
            self.connected = False
        else:
            logger.error('Tile Sleeping Failed')
            self.errorCount +=1

    def getDateTime(self):
        self.write(self.cmd['getDate'])
        header, msg = self.read(timeout = 5,
                                returnData = True,
                                headerType = '$DT',
                                debug = False)
        if (msg != None) and (len(msg)==2):
            self.gpsts = msg[0]
            self.connected = True
            if msg[1] == 'V': # The GPS is stale if its not V
                self.validDate = True
                logger.debug('DATE: %s, %s' % (self.gpsts, self.validDate))
            else:
                logger.error('Date Error')
                self.validDate = False
                self.errorCount +=1
        else:
            logger.warning('Date Timeout')
            self.connected = False
            self.validDate = False

    def getGPS(self):
        self.getDateTime()
        self.write(self.cmd['getGPS'])
        header, msg = self.read(timeout = 2,
                                returnData = True,
                                headerType = '$GN')
        if (msg != None) and (len(msg)==5):
            self.lat = float(msg[0])
            self.lon = float(msg[1])
            self.altitude = float(msg[2])
            self.course = float(msg[3])
            if (len(msg) == 4) and msg[4] !='':
                self.speed = float(msg[4])
            else:
                self.speed = 0
            self.connected = True
            self.validFix = True
            logger.debug('GPS: %f, %f' % (self.lat, self.lon))
        else:
            logger.warning('GPS Timeout')
            self.connected = False
            self.validFix = False

    def write(self, msg, timeout = 4):
        cbytes = msg.encode()
        cs = 0
        for c in cbytes[1:]:
            cs = cs ^ c
        msgBytes =  cbytes + b'*%02X\n'%cs
        self.uart.write(b'\n' + msgBytes)

    def read(self, timeout = 4, returnData = False, headerType = None, debug = False):
        self.uart.reset_input_buffer()
        uartTimeout = time.monotonic() + timeout
        header = None; msg = None
        while (uartTimeout > time.monotonic()):
            data = self.uart.readline()
            if (data != None):
                header, msg = self.parse(data)
                if (header == headerType) or (headerType == None):
                    break
        if debug:
            print(msg)
        if returnData:
            if headerType is not None and (header != headerType):
                return None, None
            else:
                return header, msg

    def parse(self, line):
        # TODO: Checksum
        line = line.decode().strip()
        header = line[0:3]
        checksum = line[-2:]
        msg = line[4:-3].split(',')
        if 'OK' in msg:
            logger.info('Tile ACK')
        return header, msg

# We don't do cryptography on this end!
# TODO: We should do a checksum
def encode_packet(packet):
    packet_hex = ''.join(['{:02x}'.format(b) for b in packet])
    if len(packet_hex) <=48:
        logger.info('LoRa Packet Processed')
        return packet_hex
    else:
        logger.warning('LoRa Packet Malformed')
        return None

# Parse int ts to set rtc
def set_rtc(ts):
    ts = str(ts)
    year = int(ts[0:4])
    month = int(ts[4:6])
    day = int(ts[6:8])
    hour = int(ts[8:10])
    minute = int(ts[10:12])
    second = int(ts[12:14])
    r.datetime = time.struct_time((year, month, day,
                                   hour, minute, second,
                                   -1,-1,-1))

# Produce a swarm style ts from rtc
def rtc_to_swarm():
    ct = r.datetime
    ts_vars = [ct.tm_year, ct.tm_mon, ct.tm_mday,
               ct.tm_hour, ct.tm_min, ct.tm_sec]
    ts_out = []
    for tidx, tvar in enumerate(ts_vars):
        tvar = str(tvar)
        if tidx == 0:
            ts_out.append(tvar)
        else:
            if len(tvar)==1:
                ts_out.append('0'+tvar)
            else:
                ts_out.append(tvar)
    return ''.join(ts_out)

# Init voltage monitoring, tile connection, rtc and lora
ina = INA(i2c)
tile = TILE()
r = rtc.RTC()
lora = LORA(spi, reset, cs)

async def listen_packet(lora):
    packet = None
    while True:
        packet = lora.rfm9x.receive(with_header=True)
        if packet is not None:
            lora.add_packet(rtc_to_swarm(), packet)
            logger.info('LoRa Packet Recieved')
        await asyncio.sleep(0)

async def process_data(lora, ina, tile):
    while True:
        while len(lora.msg)>0:
            packet_hex = encode_packet(lora.msg.pop(0))
            packet_ts = lora.ts.pop(0)
            if (packet_hex is not None) and (tile.validFix):
                data_string = '{},{},{},{},{},{},{}'.format(tile.gpsts,
                                                            tile.lat,
                                                            tile.lon,
                                                            ina.bus_v[0],
                                                            ina.bus_v[1],
                                                            packet_ts,
                                                            packet_hex)
                tile.transmit(data_string)
                logger.debug(data_string)
            else:
                logger.warning('Stale GPS')
        await asyncio.sleep(random.randint(200,300))

# Make the amount of time between updates random to
# ensure we don't consistently colide with beacons
async def update_status(ina, tile):
    while True:
        w.feed()
        tile.getGPS()
        ina.update()
        if tile.validDate:
            set_rtc(tile.gpsts)
        if (tile.connected) and (tile.validFix):
            pixels[0] = (0, 50, 0, 0)
            pixels[1] = (0, 50, 0, 0)
        elif (tile.connected) and (not tile.validFix):
            pixels[0] = (0, 50, 0, 0)
            pixels[1] = (50, 0, 0, 0)
        elif (tile.asleep):
            pixels[0] = (50, 50, 0, 0)
            pixels[1] = (50, 50, 0, 0)
        else:
            pixels[0] = (50, 0, 0, 0)
            pixels[1] = (50, 0, 0, 0)
        pixels.write()
        await asyncio.sleep(random.randint(200,300))

async def main():
    packet_task = asyncio.create_task(listen_packet(lora))
    status_task = asyncio.create_task(update_status(ina, tile))
    process_task = asyncio.create_task(process_data(lora, ina, tile))
    await asyncio.gather(packet_task, status_task, process_task)

asyncio.run(main())
