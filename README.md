# Vertical Sample Holder/PID Heater
A homemade PID heater built so as to not impede access to a vertically-held sample for fluorescence thermometry calibration.

<p align="center">
  <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/VertSampHoldPID/VSH-rend5.png" alt="Complete1" align="center" width="500" />
  <p align="center"> 3D render of the sample holder without heater (left) and with heater fully assembled (right)  </p>
</p>


The PID heater was described in the [appendix of my dissertation](https://github.com/Swicano/swicano.github.io/blob/master/images/VertSampHoldPID/DissertAppendixB2.pdf), but is comprised of 4 major components:
* heating/holding area (rendered above, 1.2 <span>&#8486;</span> worth of 22 gauge nichrome wire)
* Power supply (a repurposed ATX power supply)
* Power Modulation circuitry (an Off-The-Shelf step down converter)
* Logic/control/communication circuitry (based on an Atmel ATTiny85 MCU)

<p align="center">
  <img src="https://raw.githubusercontent.com/Swicano/swicano.github.io/master/images/VertSampHoldPID/IMG_20190618_130202.jpg" alt="Complete1" align="center" width="500" />
  <p align="center"> The PSU, Power Modulation and Logic circuitry of the PID heater </p>
</p>

Off-the-shelf components were used for the high power parts of the device so as to increase the safety, as well as to save time and money. Safety was a major concern since this system was intended to be used without direct oversight (though there was a webcam in the room as an added precaution).

The microcontroller code accomplishes the tasks of measuring the sample temperature (using a MAX31850 Thermocouple reader IC over 1-Wire) and driving the step-down converter (XLSEMI XL4016 based) appropriately to hold a set sample temperature. In addition, it acts as an I2C slave, and communicates (both change the target temperature setpoint, and report temperature measurements) with an external computer which runs the entire experiment. The Arduino code is in presented in this repository. 

The vertical sample holder was built to replace a previous iteration which consisted of a vertical copper bar sitting on a hot plate. The previous copper bar had significant temperature variation both across the sample face, as well as depending on the air flow patterns set up bu the hot plate on any particular day. As well, it was manual control only and required entering the (pitch black) room and potentially bumping the sample while adjusting the set temperature.

Schematics and mechanical details are outlined in the appendix linked above, but were not formalized and the device never left a protoboard phase.

<p align="center">
    <p align="left"><img src="https://raw.githubusercontent.com/Swicano/swicano.github.io/master/images/VertSampHoldPID/20141020_210743.jpg" align="left" height="396"/></p>
    <p align="right"><img src="https://raw.githubusercontent.com/Swicano/swicano.github.io/master/images/VertSampHoldPID/20150108_123323.jpg" alt="hot plate" align="right" height="396"/></p>

<p align="center"> The components of the old vertical sample holder and heater</p>
</p>

In regards to both of these goals, communication over I2C allowed the temperature to be changed without disruption, and the temperature variation was reduced to 2 degrees acress the sample face both front and back. 
