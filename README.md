# HomeBot
The HomeBot Project Documentation

### Contents

[About](##about)

[System Schematics](##systemschematics)

[B.O.M](##bom)

## About
HomeBot is a concept robot, designed with an innovative structure that allows it to walk, and grasp using the same mechanism, making it a much cheaper mobile manipulator than the alternatives.

We believe GPT and other advanced AI models might accelerate the development of the emerging field of service robots, creating a demand for new mobile manipulators that are more affordable than the conventional solutions used in industries. This platform could be highly accessible and enable household robots to become more widespread in our daily lives.

The structure is similar to a robotic arm, featuring 6 degrees of freedom and a gripper. However, by modifying the configuration of the motors, it gains the ability to utilize its shape to walk on two legs, climb stairs, and effectively manipulate objects.
By utilizing multiple sensors and a camera, we were able to provide it with some autonomous capabilities. One example is using computer vision to locate objects and autonomously navigate to them. Another example is the ability to fetch different nearby objects, by scanning with a distance sensor and making a grasp accordingly.

This concept platform was developed as part of our final project as Mechanical Engineering students at the Technion, 2023.


## System Schematics
### Main System
<div align="center"> 
  <img height = "350" src="./Source Code/README Images/Control Scheme.png"> 
</div>

### Electronics
<div align="center"> 
  <img height = "330" src="./Source Code/README Images/Electronics Scheme.png"> 
</div>

### Interface
<div align="center"> 
  <img height = "330" src="./Source Code/README Images/Interface Screenshot.png">
</div>

## B.O.M.
| Product | Amount | Link | 
|---------|-------|------|
|RC BAT  2200mAh 7.4 V  |  1 | [Link](https://he.aliexpress.com/item/4000173875886.html?spm=a2g0o.productlist.main.55.4e6aLTXCLTXCvx&algo_pvid=118f8981-a4c6-42c5-b909-d4c970b9f276&algo_exp_id=118f8981-a4c6-42c5-b909-d4c970b9f276-27&pdp_npi=3%40dis%21ILS%2172.64%2143.6%21%21%21%21%21%402100b0d116877615583004169d0742%2110000000606937268%21sea%21IL%210&curPageLogUid=c13Pg2rlDLkW ) |
|60 Servo kg*cm  |  4 | [Link](https://he.aliexpress.com/item/4001341870852.html?srcSns=sns_WhatsApp&spreadType=socialShare&bizType=ProductDetail&social_params=60175572503&aff_fcid=ac8520d1af984629aab0ee2ca3ff5f8d-1659955186088-08725-_uwSe5V&tt=MG&aff_fsk=_uwSe5V&aff_platform=default&sk=_uwSe5V&aff_trace_key=ac8520d1af984629aab0ee2ca3ff5f8d-1659955186088-08725-_uwSe5V&shareId=60175572503&businessType=ProductDetail&platform=AE&terminal_id=e6b9b5eecda348a780264641ee2018b3&afSmartRedirect=y&gatewayAdapt=glo2isr) |
| 20 Servo kg*cm  |  3 | [Link](https://www.aliexpress.com/item/32907625266.html?srcSns=sns_WhatsApp&spreadType=socialShare&bizType=ProductDetail&social_params=60174481688&aff_fcid=d7fd14b84f624d60be911e99c4dba091-1659955423752-04076-_vokXAP&tt=MG&aff_fsk=_vokXAP&aff_platform=default&sk=_vokXAP&aff_trace_key=d7fd14b84f624d60be911e99c4dba091-1659955423752-04076-_vokXAP&shareId=60174481688&businessType=ProductDetail&platform=AE&terminal_id=e6b9b5eecda348a780264641ee2018b3&afSmartRedirect=y) |
|Bearings |  2 | [Link](https://www.servocity.com/servoblock-standard-size-24-tooth-spline-hub-shaft/) |
|Ultrasonic Sensor  |  1 | [Link](https://www.aliexpress.com/item/1005002919950814.html?spm=a2g0o.productlist.0.0.408f753c9eelLH&algo_pvid=76c9175e-38f1-4114-846f-c36bd94508f8&algo_exp_id=76c9175e-38f1-4114-846f-c36bd94508f8-6&pdp_ext_f=%7B%22sku_id%22%3A%2212000022797597266%22%7D&pdp_npi=2%40dis%21USD%212.65%212.17%21%21%21%21%21%402100bde316599558114852327e87de%2112000022797597266%21sea&curPageLogUid=1NNzij3e4W3y) |
|IMU |  1 | [Link](https://he.aliexpress.com/item/32340949017.html?spm=a2g0o.productlist.0.0.69784a71UyfkJP&algo_pvid=a7a71383-25c7-4a1c-9dd6-62a185676d55&algo_exp_id=a7a71383-25c7-4a1c-9dd6-62a185676d55-0&pdp_ext_f=%7B%22sku_id%22%3A%2210000000609322940%22%7D&pdp_npi=2%40dis%21USD%211.35%211.19%21%21%21%21%21%402100bb4916599555964388339e9e9d%2110000000609322940%21sea&curPageLogUid=odPaV6SjwVoV&gatewayAdapt=glo2isr) |
|Arduino Uno |  1 | [Link](https://he.aliexpress.com/item/32864836449.html?spm=a2g0o.productlist.0.0.6d7a2355X5eJdk&algo_pvid=80ae9458-dd33-4f4c-8cd6-7b000cb5ea00&algo_exp_id=80ae9458-dd33-4f4c-8cd6-7b000cb5ea00-0&pdp_ext_f=%7B%22sku_id%22%3A%2212000023789267551%22%7D&pdp_npi=2%40dis%21USD%214.78%213.82%21%21%21%21%21%400b0a0ac216599559500621346e41a7%2112000023789267551%21sea&curPageLogUid=oHRCK1xAgL3m&gatewayAdapt=glo2isr) |
|Bluetooth Module |  1 | [Link](https://he.aliexpress.com/item/32786773297.html?spm=a2g0o.productlist.0.0.5e3b5aafSd0AEe&algo_pvid=18b06263-dd29-4e3e-b95d-6ed8de0a30c0&algo_exp_id=18b06263-dd29-4e3e-b95d-6ed8de0a30c0-1&pdp_ext_f=%7B%22sku_id%22%3A%2210000010469459308%22%7D&pdp_npi=2%40dis%21USD%213.3%210.01%21%21%21%21%21%402100bddf16599560372157334ee34e%2110000010469459308%21sea&curPageLogUid=gjJzOGLRD0cn&gatewayAdapt=glo2isr)|
|Voltage Regulator |  1 | [Link](https://a.aliexpress.com/_EJZAgTN) |
|esp cam |  1 | [Link](https://he.aliexpress.com/item/1005003472117545.html?spm=a2g0o.productlist.main.1.15502Dtt2Dtt0i&algo_pvid=d07eea10-2d0a-497f-9d19-72d87fc2ccae&algo_exp_id=d07eea10-2d0a-497f-9d19-72d87fc2ccae-0&pdp_npi=3%40dis%21ILS%213.59%212.89%21%21%21%21%21%402100b77316877600687485861d0753%2112000025941403906%21sea%21IL%210&curPageLogUid=mID1d383shdP) |
|Jumper Cables |  - | - |
|M3 and M4: nuts, bolts & inserts|  - | - |
|Crimping tools |  - | - |
|Soldering iron |  - | - |
