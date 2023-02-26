# TIVAC基础功能例程

## 代码依赖

### 软件

+ CCS编译器，配置TIVAWARE

### 硬件

TM4C123G单片机。

## 文件说明

### gpio.c

实现gpio端口的**输出功能**和**输入中断**。

### usart.c

实现tivac串口通讯，包括接收和发送数据。

[(14条消息) TM4C123G学习笔记（4）——串口UART_SStegosaurus的博客-CSDN博客](https://blog.csdn.net/SStegosaurus/article/details/108468819)



### timer.c

实现定时器中断配置。‘

定时器级联：[(14条消息) TM4C123系列（五）————timer定时器（timer模式）_小葛必上岸的博客-CSDN博客](https://blog.csdn.net/weixin_56003594/article/details/125662058)