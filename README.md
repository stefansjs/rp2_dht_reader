# dht_reader
PIO-based reader for DHT11 and DHT22 for use with Raspberry Pi Pico (or a RP2040 or similar mcu)

## OVERVIEW
By using PIO we can simplify the code compared to othr

## USAGE
Connect your DHT sensor to the Pico. Pull up resistor between Vcc and Data lines is not necessary, because the code enables an internal pull-up in the pico, although if you do it shouldn't hurt.
My DHT11 needed 5V on Vcc to work. DHT22 Worked fine on 3.3 Volts.
Sample usage - assuming Data line is connected to Pin 5
```
from time import time
from dht_reader import DhtReader

sensor = DhtReader(5, 'DHT11', 0)  # 0 is the number of state machine to create.
sensor.start() 
# sleep or do something else for 2 seconds
while True:
  if not sensor.err:
    if sensor.stamp < time() - 2:
      print('sensor timed out')
    else:
      humidity = sensor.humidity / 10
      temperature = sensor.temperature / 10
      print humidity, '%', temperature, '°C'
  elif sensor.err == 2:
    print('Could not read from FIFO')
  elif sensor.err == 1:
    print('Checksum check failed')
  else:
    print('This should never happen')
```

## Notes
The PIO program reads sensor data and pushes it as 5 32-bit words to FIFO (The RX and TX FIFOs are combined as Rx, to allow the 5th word)
This way bytes transmitted by DHT are cleanly separated. We wouldn't save anything by bit packing or not joining the FIFOs.
The PIO stalls waiting for a 41st bit that will not come.

Every 2 seconds a timer triggers the `sense` method, which reads FIFO, checks the checksum, sets error code, timestamp, temperature and humidity to instance properties.

Timestamp allows us to detect when the DHT has not responded for some time (by comparing with current time).
Error code tells us if soemthing went wrong with reading data.

This way temperature and humidity is avaliable all the time, even if there are transient problems with reading the sensor.
Thanks to using PIO and Timer with interrupt, the main code only has to read the object's attributes and the CPU is only busy for short moments, when the timer triggers the `sense` method.

We can use multiple sensors - just create an instance for each one with different state machine numbers (0-7).

In class instantiation, if we use "DHT22" as argument the class is set to read DHT22. If it is set to anything else, the it is set to read DHT11.

DHT11 sometimes works with 5V and not with 3.3V.

DHT is tricky to work with and PIO doesn't allow much debugging. If the code doesn't work, check with dht library bundled with micropython.
  This way you can know if it's the code's fault.
