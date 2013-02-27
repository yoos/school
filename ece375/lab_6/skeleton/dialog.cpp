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

int data1[6];
int z=0,c;
float a=0,b=300,p=0,q=300;
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

    //int addr = ??;	 //address for nunchuck

		//Access the I2C slave

    if(ioctl(file, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access to slave.\n");
        printf("error..., something is wrong %s\n", strerror(errno));
        exit(1);
    }

		//TODO: write initialization data to the nunchuk.
		//if it returns with error (-1) exit with status 1
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

						//TODO : convert the input data to the correct format

            //printf("the final data is %x\n", data1[i]);

            }

		//TODO: Convert data bytes into analog X/Y, accelerometer X/Y/Z, button X/Z
				
     // Output from the nunchuck for reference //

				//TODO: Print output of analog stick and X, Y, Z axes
        //ex.    printf("Analog X: %d\n", data1[0]);

				//TODO: Print output of buttons Z and C
				//When drawing pressing button Z should exit
				//When drawing pressing button C should draw

		//Calculate the next point based on analog stick data.	
		//You might need to convert some values.  Experiment...


		//Draw a line between the previous point and the new point.

    std::cout<<a<<" "<<b<<"\n";

}

