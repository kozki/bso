# BSO project

## Testing the BME680 sensor for indoor air quality

### Abstract
An adult breathes around 15,000 litres of air every day. When we breathe polluted air, pollutants get into our lungs; they can enter the bloodstream and be carried to our internal organs. This can cause severe health problems such as asthma, cardiovascular diseases and even cancer and reduces the quality and number of years of life. So the BME680 is a great sensor for indoor air quality control, because it is a 4-in-1 digital sensor that measures temperature, humidity, air pressure and gases in form of VOCs. It presents as a great tool to inform you about poor air quality which may affect your health in the long run. We used I2C protocol for communication between BME680 sensor and esp8266 chip with the use of bme680 driver. We also demonstrate how to use Wi-FI to transfer the data to MQTT server using I2C protocol. When measuring the air quality we turn on LED if air quality is above given threshold using I2C to communicate with PCF8574.

    #define threshold 100;
    
We used 100 as a threshold, because anything above this is known to be unhealty. In the video it is shown how the threshold is used and the LED light starts flashing.

## Demo
https://drive.google.com/file/d/1po4968wMF3BUL7tamgRRlYVZJqJiJH0J/view?usp=sharing


## Graphs
### Office 
![Alt text](./graphs/office_new.png?raw=true)

### Bedroom
![Alt text](./graphs/bedroom_new.png?raw=true)

### Living room
![Alt text](./graphs/living_room.png?raw=true)

### Testing the sensor
![Alt text](./graphs/sensor_test.png?raw=true)


We can see from the graphs that the values were under the healthy threshold, unless we tried to test it as we did in the last graph, when we blew on the sensor, just to show what happens if the pollutants are present. In that case we trigger the LED on the board to start flashing.
