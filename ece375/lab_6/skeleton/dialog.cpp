#include "dialog.h"
#include "ui_dialog.h"
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

//Skeleton Code for ECE 375 Lab 6 - Intel Atom Lab

int file;
int adapter_nr = 14;	//dynamically determined; in our system, i2cdetect -l shows i2c0 is for smbus (SMBus I801 adapter at 0400)
char filename[20];

int datal[6];   // Raw sensor reads
float dataf[5];   // Sensor reads mapped to [-1, 1]
int z=0,c;
int xa=0, xb=0, ya=0, yb=0;
int k=1;

QPen pen;

void openfile()
{
    //open the device file
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);

    if (file < 0)
    {
        printf("Failed to open i2c device.\n");
        exit(1);
    }

    //specify what device address you want to communicate

    int addr = 0x52;	 //address for nunchuck

		//Access the I2C slave

    if(ioctl(file, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access to slave.\n");
        printf("error..., something is wrong %s\n", strerror(errno));
        exit(1);
    }

		//TODO: write initialization data to the nunchuk.
        //if it returns with error (-1) exit with status 1\

    // Initialize white nunchuck
//    if(i2c_smbus_write_byte(file, 0x40) == -1) {
//        printf("error..., something is wrong  gvcn jhbv %s\n", strerror(errno));
//        exit(1);
//    }
//    if(i2c_smbus_write_byte(file, 0x00) == -1) {
//        printf("error..., something is wrong %s\n", strerror(errno));
//        exit(1);
//    }

    if(i2c_smbus_write_byte_data(file, 0x40, 0x00) == -1)
    {
        printf("error..., something is wrong %s\n", strerror(errno));
        exit(1);
    }

    if(i2c_smbus_write_byte(file, 0x00) == -1) {
        printf("error..., something is wrong %s\n", strerror(errno));
        exit(1);
    }

}

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    openfile();
    timer= new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(repaint()));

    timer->start(100);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::paintEvent(QPaintEvent *e)
{
    unsigned char data[6];

    QPainter painter(this);
    QWidget::setAttribute( Qt::WA_OpaquePaintEvent, true );

       if(i2c_smbus_write_byte(file, 0x00) == -1)
        {
            printf("error..., something is wrong %s\n", strerror(errno));
            exit(1);
        }

        for(int i = 0; i < 6; ++i)
            {

						//TODO: read a byte from I2C into the data array
                data[i] = (unsigned char) i2c_smbus_read_byte(file);
						//TODO : convert the input data to the correct format
                data[i] = (data[i] ^ 0x17) + 0x17;
                //printf("the final data is %x\n", data[i]);

            }

		//TODO: Convert data bytes into analog X/Y, accelerometer X/Y/Z, button X/Z
        datal[0] = data[0];
        datal[1] = data[1];
        datal[2] = (data[2] << 2) | (data[5] & 0x0c);
        datal[3] = (data[3] << 2) | (data[5] & 0x30);
        datal[4] = (data[4] << 2) | (data[5] & 0xc0);

        z = (data[5] & 0x01);
        c = (data[5] & 0x02) >> 1;

     // Output from the nunchuck for reference //

				//TODO: Print output of analog stick and X, Y, Z axes
        printf("Analog X: %d\n", datal[0]);
        printf("Analog Y: %d\n", datal[1]);
//        printf("Accel X: %d\n", datal[2]);
//        printf("Accel Y: %d\n", datal[3]);
//        printf("Accel Z: %d\n", datal[4]);

				//TODO: Print output of buttons Z and C
//        printf("Button Z: %d\n", z);
//        printf("Button C: %d\n", c);


		//Calculate the next point based on analog stick data.	
		//You might need to convert some values.  Experiment...
        dataf[0] = (((data[0])-23) * 2.0)/(217-23) - 1.0;
        dataf[1] = (((data[1])-32) * 2.0)/(235-32) - 1.0;

        //printf("AX: %f\n", dataf[0]);
        //printf("AY: %f\n", dataf[1]);

		//Draw a line between the previous point and the new point.
        xa = xb;
        ya = yb;
        xb += (int) (30*dataf[0]);
        yb -= (int) (30*dataf[1]);
        xb = (xb < 0) ? 0 : xb;
        xb = (xb > 300) ? 300 : xb;
        yb = (yb < 0) ? 0 : yb;
        yb = (yb > 300) ? 300 : yb;

        //When drawing pressing button C should draw
if(!c) {
    pen.setColor(QColor::fromRgb(255,0,0));
    painter.setPen(pen);
}

        printf("AX: %f  AY: %f  +x: %d  +y: %d\n", dataf[0], dataf[1], (int) (5*dataf[0]), (int) (5*dataf[1]));

        //printf("%d %d %d %d\n", xa, ya, xb, yb);

        painter.drawLine(xa, ya, xb, yb);

        //When drawing pressing button Z should exit
if(!z) {
    painter.eraseRect(0,0, this->width(), this->height());
}


    //std::cout<<a<<" "<<b<<"\n";

}

