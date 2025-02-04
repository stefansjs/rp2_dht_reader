# https://docs.micropython.org/en/latest/library/rp2.html#pio-related-functions
# https://www.ocfreaks.com/basics-interfacing-dht11-dht22-humidity-temperature-sensor-mcu/
from array import array
from time import sleep, time
import micropython
from machine import Pin, Timer
import rp2

READ_PERIOD = const(1_000)
PIO_FREQUENCY = const(200_000)
LED_PIN = const(25)

"""
set_init: we define the single pin, and assign a high output to it to signal IDLE to DHT
autopush=True, push_thresh=8 : these push to RX FIFO after reading 8 bits into the ISR,
    which saves us some cycles and instructions. According to RP2040 datasheet the push happens on an IN instruction (chapter 3.5.4)
fifo_join=rp2.PIO.JOIN_RX : use BOTH FIFOs as a single RX FIFO of 8 words (we use 5, but it's one more than non-joined rx_fifo depth)
We could do with 4 words and a blocked push waiting for a `get()`
But since we don't transmit to state machine, then tx fifo would go unused.
"""
@rp2.asm_pio(set_init=rp2.PIO.OUT_HIGH, fifo_join=rp2.PIO.JOIN_RX, autopush=True, push_thresh=8)
def read_dht():
    # use a trick to place numbers larger than 31 in a register.
    mov(osr, invert(null))  # get a source of 1 bits
    out(x, 7)  # 1111111 = 127 shift some bits into the register to get the larger value
    set(pindirs, 1)
    set(pins, 0)
    # 31 cycles * 127 loops * 5 us =  19.685 ms... over 18 ms as required by the sensor
    label('long_wait')
    jmp(x_dec, 'long_wait') [31]
    set(pins, 1) [4]
    # set pin to read mode
    set(pindirs, 0) [4]
    # wait for low, then high => a rising edge
    wait(0, pin, 0) [10]
    wait(1, pin, 0) [10]
    # after a 1 jump here, because there will be a low state coming, before a high one
    label('read_after_1')
    wait(0, pin, 0) [7]
    # after a 0 jump here, because only high state is coming.
    # We save some instructions and cycles by using wrap_target
    wrap_target()
    wait(1, pin, 0) [5]  # 35us after high
    in_(pins, 1)  # 40us after high
    jmp(pin, 'read_after_1') #50us after high
    # if we get here, we WRAP to wrap_target (meaning we read a 0)
    # after DHT sensor has transmitted 40 bits the line will be pulled high,
    # so we may read an additional 1, jump to label, and remain waiting for a low.
    # the additional bit will remain in isr - one bit is not enugh for autopush
    # and will be cleared when state_machine.reset() is called from sense function


class DhtReader:
    """
    TODO: readme
    """
    def __init__(self, sensor_pin: int, sensor_model: str, state_machine_num: int) -> None: 
        self.pin = Pin(sensor_pin, Pin.IN, Pin.PULL_UP)
        self.model: str = sensor_model
        self.humidity: int = -100
        self.temperature: int = -100
        self.stamp: int = -100
        self.err: int = 0
        self.sm = rp2.StateMachine(state_machine_num, read_dht, freq=PIO_FREQUENCY,
                                   in_base=self.pin, set_base=self.pin, jmp_pin=self.pin)
        self.repeater = Timer()
        
    def sense(self, _):
        readout = array('L', list(range(5)))
        if not self.sm.rx_fifo():
            self.err = 2
        else:        
            for i in range(5):
                readout[i] = self.sm.get()
        self.sm.restart()
        checksum = readout[4]
        if (sum(readout[0:4]) & 0xFF) == checksum:
            if self.model == 'DHT22':
                self.humidity = readout[0] * 0xFF + readout[1]
                t_sign = (readout[2] >> 7) * -2 + 1  #translate first bit to -1 or 1
                self.temperature = t_sign * ((readout[2] & 0x7F) * 0xFF + readout[3])
            else:
                self.humidity = readout[0] * 10
                self.temperature = readout[2] * 10
            self.stamp = int(time())
            self.err = 0
        else:
            self.err = 1

    def start(self):
        self.sm.active(1)
        self.repeater.init(
                period=READ_PERIOD,
                mode=Timer.PERIODIC,
                callback=lambda t: micropython.schedule(self.sense, None)
            )


def blink(num, light):
    light.value(0)
    light_time = const(0.03)
    dark_time = const(0.1)
    break_time = dark_time * 2
    digits=[]
    while num:
        num, digit = divmod(num, 10)
        digits.append(digit)
    for digit in digits[::-1]:
        for i in range(digit):
            light.value(1)
            sleep(light_time)
            light.value(0)
            sleep(dark_time)
        sleep(break_time)
    sleep(break_time)

if __name__ == '__main__':
    led = Pin(LED_PIN, Pin.OUT)
    sensor1 = DhtReader(0, 'DHT11', 0)
    sensor2 = DhtReader(1, 'DHT22', 1)
    led.value(1)
    sleep(0.5)
    led.value(0)
    sensor1.start()
    sensor2.start()

    while True:
        sleep(1)
        print(
            "H:", sensor1.humidity/10,
            "H2:", sensor2.humidity/10,
            "T:", sensor1.temperature/10,
            "T2:", sensor2.temperature/10
            )
        if sensor2.err:
            blink(sensor2.err, led)
