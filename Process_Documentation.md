**23.11.2025** ***Coming up with an idea:***

Before coming up with an initial idea for my final project I already knew that I would like to make something that I or someone around me could actually make use of. 
While doing some research online and on you tube I stumbled across many different Arduino projects regarding bicycles. 
After giving the topic some thought I came up with the initial idea of creating a silent security device. 
The device was supposed to call or send a text message to the phone of the user when the bicycle is being used. 

**26.11.2025** ***Nailing down my idea:***

After having a general idea of what I would like to do for my final project and what components I would need to use (f.e. an ESP32 as the central microcontroller),
I’ve send an E-mail to my professor to get some feedback and a better idea on how realistic this project would be for me to do.

**09.12.2025** ***First presentation:***

In the beginning of december I held a presentation infront of the class and the professor explaining my initial project idea. 
I had already done quite some research on google, You Tube and asking AI, my Dad and my professor. Through that research I was able to layout:
- A technical concep
- Hardwear components 
- Softwear components & libraries
- Potential Refrences
- Challanges & open questions
- And additional functions (like GPS tracking) which I could include in the future

My presentation: 

https://www.canva.com/design/DAG68Z0ASbo/kgA9yAEKSYflQ0bWsIWqxA/view?utm_content=DAG68Z0ASbo&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hee3be1c131

**13.-14.12.2025** ***Finishing the project concept:***

The weekend after my presentation I drove to my dads place to review the previous feedback I had gotten from my professor. Because of his knowledge regarding coding and hardware he was able
to help me coming up with new ways and adjustments to my initial project idea. After the weekend I had my final project abstract done 
(little did I know that I would actually still have to change quite some things along the way🫣). 

Peoject Lineout:

https://docs.google.com/document/d/1CNGolNSGCiXH2bkl5u_3fEt8SCGP3HFgc-hNApr6zvI/edit?usp=sharing

**04.01.2026** ***Collecting all equipment needed for the project:***

In the beginning of the new year I’ve ordered all of the hardware equipment that I would need. By now I had already done some changes on the project plan again 
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

After all of the equipment arrived I’ve took the weekend to build my prototype and afterwards review it with my dad for further feedback.
For the Prototype I used the breadboard included in the Arduino Kit and an additional Breadboard because the set up did take up quite some space. I’ve used the ESP32, a Buzzer, 
the SD-Card reader and the movement detector and connected everything using the jumping wires from my Kit.

I’ve also took a look at where I could locate the security device on the bicycle and landed on the beach of the saddle since there it won’t bother the user while driving
on the bicycle and it is easy to reach on order to turn it on and of or charge it. That meant though that I wouldn’t have much space and that for the device had to be small. 
After telling my dad about the problem he then suggested for me to use a perforated grid board instead of the bread boards and connect everything with U shape jumping wires. 
Further changes I’ve decided to make after reviewing the prototype include:
- Using a USB-Powerbank instead of battery monitoring via analog voltage measurement
- Instead of a button interface for arming and disarming the device, using a RFID 

**So I had to order some more things:**
- Perforated grid board
- USB-Power bank 
- RFID
- U shape breadbord jumping wires
- Additional Buzzer (Because I wanted to keep the final product)

The Prototype:


**17.-18.01.2026** ***Building the product:***

The next weekend I finally build the actual product step by step:
1. Translating the connections from the prototype to the actual product
2. Solder everything together (ESP32, RFID, SD card reader, buzzer, movement detector) 
3. Create the software 
- Bla Bla

After I was done with all of the steps above I started thinking about possible production cases could put the device into to attach it to the bicycle and have it somewhat protected.
The two ideas I came up with were:
1. Designing a case myself and printing it with a 3D printer
2. Buying a bicycle saddle bag myself and doing my own little changes to it to make it safer

**27.01.2026** ***Intermediate Presentation:***

At the end of January I held my second presentation showing the prototype in front of the class and my professor. The most important take away from that presentation was my professor's 
feedback on the type of case that I should use for the device. So right after the lesson I started looking into different saddle cases that I could buy instead of printing my own.

My presentation:

https://www.canva.com/design/DAG_VIViMn0/8sIeP5AGplpsw27RB_pZcQ/view?utm_content=DAG_VIViMn0&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hd8ca1cca40

**28.01-08.02.2026** ***Finding the perfect Case:***

While looking at different cases on the internet I quickly realized, that there where quite some things I had to look out for when trying to find the perfect case:
- The price (some where really pricy)
- The size (not too big but also not to small)
- That it had a sipper that i could fit a lock through to make it even safer
- The way it attached to the bicycle (no velcro fastener because it is to easy to detach from the bisycle)

**10.-11.02.2026** ***The final steps:***

After the case finally arrived I just had to do some more final steps:
✅DIY some adjustments to the case (add a lock and attach the case with cable ties)
✅Make the Demo Video
✅Come up with a fun name

And just like that after many weeks of work and a lot of try and error (especially while creating the software component) the **“StayAwayItsMyBike”** security device was finally done.
