**23.11.2025** ***Coming up with an idea:***

Before coming up with an initial idea for my final project I already knew that I would like to make something that I or someone around me could actually make use of. 
While doing some research online and on You Tube I stumbled across many different Arduino projects regarding bicycles and alarm systems. 
After giving the topic some thought I came up with the initial idea of creating a silent security device. 
The device was supposed to call or send a text message to the phone of the user when the bicycle is being used. 

**26.11.2025** ***Nailing down my idea:***

After having a general idea of what I would like to do for my final project and what components I would need to use (f.e. an ESP32 as the central microcontroller),
I’ve send an E-mail to my professor to get some feedback and a better idea on how realistic this project would be for me to do.

**09.12.2025** ***First presentation:***

In the beginning of december I held a presentation infront of the class and the professor explaining my initial project idea. 
I had already done quite some research on google, You Tube and asking AI, my Dad and my professor. Through that research I was able to layout:
- A technical concept
- Hardwear components 
- Softwear components and libraries
- Potential Refrences
- Challanges and open questions
- And additional functions (like GPS tracking) which I could include in the future

My presentation: 

https://www.canva.com/design/DAG68Z0ASbo/kgA9yAEKSYflQ0bWsIWqxA/view?utm_content=DAG68Z0ASbo&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hee3be1c131

**13.-14.12.2025** ***Finishing the project concept:***

The weekend after my presentation I drove to my dads place to review the previous feedback I had gotten from my professor. Because of his knowledge regarding coding and hardware he was able
to help me come up with new ways and adjustments to my initial project idea. After the weekend I had my final project abstract done 
(little did I know that I would actually still have to change quite some things along the way🫣). 

Project Lineout:

https://docs.google.com/document/d/1CNGolNSGCiXH2bkl5u_3fEt8SCGP3HFgc-hNApr6zvI/edit?usp=sharing

**04.01.2026** ***Collecting all equipment needed for the project:***

In the beginning of the new year I’ve ordered all of the hardware equipment that I would need. By now I had already done some changes on the project plan 
and decided to include a loud alarm aswell as sending the notification via E-Mail instead of a HTTP notification. 
**Hardware included in the Arduino kit:**
- Tilt switch
- Buttons (arming/disarming the 
- LED’s & Resistors
- Jumper wires 
- Breadboard
- Buzzer
**Hardware I had to buy:**
- An additional Breadboard
- EPS32 development board
- SD-card reader
- Acceleration sensor/gyroscope

**10.-11.01.2026** ***Building the prototype:***

After all of the equipment arrived I’ve took the weekend to build my prototype and afterwards reviewed it with my dad for further feedback.
For the Prototype I used the breadboard included in the Arduino kit and an additional breadboard because the set up did take up quite some space. I’ve used the ESP32, a Buzzer, 
the SD-Card reader and the movement detector and connected everything using the jumping wires from my kit.

I’ve also took a look at where I could locate the security device on the bicycle and landed on the back of the saddle, hoping that this way it wouldn't bother the user while driving
on the bicycle. In addition it made it easier the device when wanting to turn it on and of or charge it. That meant though that I would have limitted storage space for the device which thatfore had to be small. 
After telling my dad about the problem he then suggested for me to use a perforated grid board instead of the bread boards and connect everything with U shape jumping wires. 
Further changes I’ve decided to make after reviewing the prototype include:
- Using a USB-Powerbank instead of battery monitoring via analog voltage measurement
- Using a RFIDI nstead of a button interface to arm and disarm the device
- Adding a RGB LED to signal the devices state to the user

**So I had to order some more things:**
- Perforated grid board
- USB-Powerbank 
- RFID
- U shape breadbord jumping wires
- Additional Buzzer (because I wanted to keep the final product)

I started testing the behavior of the components on the breadboard with simple sketches to make sure each component is working properly. At this point I realized that when having more then one component that is using the SPI interface, those components need to share the same GPIOs. In my case it was the SD card module and the RFID module. I decided to solve it by using the SD card only at the very first moments just after booting, then deactivating the SD card module and reusing the same SPI GPIOs for the RFID module for the rest of the runtime.

The Prototype:
![imgae alt](https://github.com/RonjaVerkauf/TechBasics2_FinalProject_RonjaVerkauf/blob/10e3a46fad8a5ab3dc9e01422ae9fa74129893b1/Media/IMG_7505.HEIC)

**17.-18.01.2026** ***Building the product:***

The next weekend I finally build the actual product step by step:
1. Translating the connections from the prototype to the actual product
2. Solder everything together (ESP32, RFID, SD card reader, movement detector, buzzer, RGB LED) 
3. Create the software 

After I was done with all of the steps above I started thinking about possible protection cases.
The two ideas I came up with were:
1. Designing a case myself and printing it with a 3D printer
2. Buying a bicycle saddle bag and adding my own adjustments to it to make it safer

**27.01.2026** ***Intermediate Presentation:***

At the end of January I held my second presentation showing. The most important take away from that presentation was my professor's 
feedback on the type of case that I should use for the device. So right after the lesson I started looking into different saddle cases that I could buy instead of printing my own.

My presentation:

https://www.canva.com/design/DAG_VIViMn0/8sIeP5AGplpsw27RB_pZcQ/view?utm_content=DAG_VIViMn0&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hd8ca1cca40

**28.01-08.02.2026** ***Finding the perfect Case:***

While looking at different cases on the internet I quickly realized, that there where quite some things I had to look out for when trying to find the right case:
- The price (some where really pricy)
- The right size
- The way it attached to the bicycle
- The fabric (waterproof but soft so that I could cut into it to make adjustments)

**10.-11.02.2026** ***The final steps:***

After the case finally arrived I just had to do some more final steps:

✅DIY the adjustments to the case 

✅Make the Demo Video

✅Come up with a fun name

While making the demo video I was actually using the device for the first time "in the field" and for a quite long time over and over again. At this point I realized that the time it takes for the notifications to arrive at the user per e-mail is too unpredictable and most of the times also too long. The optimal solution would surely be to use a GSM module and notify the user by SMS messages or push notification to a mobile app. But that would exceed the financial limits of my project. I decided as an interim solution to send the user sms messages by using the REST-API of a SMS serivce provider. I decided to use ClickSend because they offer free credits for developers who want to test their service. And indeed the SMS notifications were not only much faster, but also the duration between sending and receiving the message was much more constant. Making the changes to the sketch actually very easy and the logic is streight forward.

And just like that after many weeks of work and a lot of try and error (especially while creating the software component) the **“StayAwayItsMyBike”** security device was finally done.
