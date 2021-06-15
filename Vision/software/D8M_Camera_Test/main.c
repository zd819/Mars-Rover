#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "I2C_core.h"
#include "terasic_includes.h"
#include "mipi_camera_config.h"
#include "mipi_bridge_config.h"

#include "auto_focus.h"

#include <fcntl.h>
#include <unistd.h>

//EEE_IMGPROC defines
#define EEE_IMGPROC_MSG_START ('R'<<16 | 'B'<<8 | 'B')

//offsets
#define EEE_IMGPROC_STATUS 0
#define EEE_IMGPROC_MSG 1
#define EEE_IMGPROC_ID 2
#define EEE_IMGPROC_BBCOL 3

#define EXPOSURE_INIT 0x002000
#define EXPOSURE_STEP 0x100
#define GAIN_INIT 0x080
#define GAIN_STEP 0x040
#define DEFAULT_LEVEL 3

#define MIPI_REG_PHYClkCtl		0x0056
#define MIPI_REG_PHYData0Ctl	0x0058
#define MIPI_REG_PHYData1Ctl	0x005A
#define MIPI_REG_PHYData2Ctl	0x005C
#define MIPI_REG_PHYData3Ctl	0x005E
#define MIPI_REG_PHYTimDly		0x0060
#define MIPI_REG_PHYSta			0x0062
#define MIPI_REG_CSIStatus		0x0064
#define MIPI_REG_CSIErrEn		0x0066
#define MIPI_REG_MDLSynErr		0x0068
#define MIPI_REG_FrmErrCnt		0x0080
#define MIPI_REG_MDLErrCnt		0x0090

void mipi_clear_error(void){
	MipiBridgeRegWrite(MIPI_REG_CSIStatus,0x01FF); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLSynErr,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_FrmErrCnt,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLErrCnt, 0x0000); // clear error

  	MipiBridgeRegWrite(0x0082,0x00);
  	MipiBridgeRegWrite(0x0084,0x00);
  	MipiBridgeRegWrite(0x0086,0x00);
  	MipiBridgeRegWrite(0x0088,0x00);
  	MipiBridgeRegWrite(0x008A,0x00);
  	MipiBridgeRegWrite(0x008C,0x00);
  	MipiBridgeRegWrite(0x008E,0x00);
  	MipiBridgeRegWrite(0x0090,0x00);
}

void mipi_show_error_info(void){

	alt_u16 PHY_status, SCI_status, MDLSynErr, FrmErrCnt, MDLErrCnt;

	PHY_status = MipiBridgeRegRead(MIPI_REG_PHYSta);
	SCI_status = MipiBridgeRegRead(MIPI_REG_CSIStatus);
	MDLSynErr = MipiBridgeRegRead(MIPI_REG_MDLSynErr);
	FrmErrCnt = MipiBridgeRegRead(MIPI_REG_FrmErrCnt);
	MDLErrCnt = MipiBridgeRegRead(MIPI_REG_MDLErrCnt);
	printf("PHY_status=%xh, CSI_status=%xh, MDLSynErr=%xh, FrmErrCnt=%xh, MDLErrCnt=%xh\r\n", PHY_status, SCI_status, MDLSynErr,FrmErrCnt, MDLErrCnt);
}

void mipi_show_error_info_more(void){
    printf("FrmErrCnt = %d\n",MipiBridgeRegRead(0x0080));
    printf("CRCErrCnt = %d\n",MipiBridgeRegRead(0x0082));
    printf("CorErrCnt = %d\n",MipiBridgeRegRead(0x0084));
    printf("HdrErrCnt = %d\n",MipiBridgeRegRead(0x0086));
    printf("EIDErrCnt = %d\n",MipiBridgeRegRead(0x0088));
    printf("CtlErrCnt = %d\n",MipiBridgeRegRead(0x008A));
    printf("SoTErrCnt = %d\n",MipiBridgeRegRead(0x008C));
    printf("SynErrCnt = %d\n",MipiBridgeRegRead(0x008E));
    printf("MDLErrCnt = %d\n",MipiBridgeRegRead(0x0090));
    printf("FIFOSTATUS = %d\n",MipiBridgeRegRead(0x00F8));
    printf("DataType = 0x%04x\n",MipiBridgeRegRead(0x006A));
    printf("CSIPktLen = %d\n",MipiBridgeRegRead(0x006E));
}



bool MIPI_Init(void){
	bool bSuccess;


	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_MIPI_BASE, 50*1000*1000,400*1000); //I2C: 400K
	if (!bSuccess)
		printf("failed to init MIPI- Bridge i2c\r\n");

    usleep(50*1000);
    MipiBridgeInit();

    usleep(500*1000);

//	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_CAMERA_BASE, 50*1000*1000,400*1000); //I2C: 400K
//	if (!bSuccess)
//		printf("failed to init MIPI- Camera i2c\r\n");

    MipiCameraInit();
    MIPI_BIN_LEVEL(DEFAULT_LEVEL);
//    OV8865_FOCUS_Move_to(340);

//    oc_i2c_uninit(I2C_OPENCORES_CAMERA_BASE);  // Release I2C bus , due to two I2C master shared!


 	usleep(1000);


//    oc_i2c_uninit(I2C_OPENCORES_MIPI_BASE);

	return bSuccess;
}

//float round(float number, int multiple){
//	//https://stackoverflow.com/a/3407254
//	int remainder = number % multiple;
//	if(remainder == 0){
//		return number;
//	}else{
//		return number + multiple - remainder;
//	}
//}


int main()
{
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

  printf("DE10-LITE D8M VGA Demo\n");
  printf("Imperial College EEE2 Project version\n");
  IOWR(MIPI_PWDN_N_BASE, 0x00, 0x00);
  IOWR(MIPI_RESET_N_BASE, 0x00, 0x00);

  usleep(2000);
  IOWR(MIPI_PWDN_N_BASE, 0x00, 0xFF);
  usleep(2000);
  IOWR(MIPI_RESET_N_BASE, 0x00, 0xFF);

  printf("Image Processor ID: %x\n",IORD(0x42000,EEE_IMGPROC_ID));
  //printf("Image Processor ID: %x\n",IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_ID)); //Don't know why this doesn't work - definition is in system.h in BSP


  usleep(2000);


  // MIPI Init
   if (!MIPI_Init()){
	  printf("MIPI_Init Init failed!\r\n");
  }else{
	  printf("MIPI_Init Init successfully!\r\n");
  }

//   while(1){
 	    mipi_clear_error();
	 	usleep(50*1000);
 	    mipi_clear_error();
	 	usleep(1000*1000);
	    mipi_show_error_info();
//	    mipi_show_error_info_more();
	    printf("\n");
//   }


#if 0  // focus sweep
	    printf("\nFocus sweep\n");
 	 	alt_u16 ii= 350;
 	    alt_u8  dir = 0;
 	 	while(1){
 	 		if(ii< 50) dir = 1;
 	 		else if (ii> 1000) dir =0;

 	 		if(dir) ii += 20;
 	 		else    ii -= 20;

 	    	printf("%d\n",ii);
 	     OV8865_FOCUS_Move_to(ii);
 	     usleep(50*1000);
 	    }
#endif






    //////////////////////////////////////////////////////////
        alt_u16 bin_level = DEFAULT_LEVEL;
        alt_u8  manual_focus_step = 10;
        alt_u16  current_focus = 300;
    	int boundingBoxColour = 0;
    	alt_u32 exposureTime = EXPOSURE_INIT;
    	alt_u16 gain = GAIN_INIT;

        OV8865SetExposure(exposureTime);
        OV8865SetGain(gain);
        Focus_Init();
  while(1){

       // touch KEY0 to trigger Auto focus
	   if((IORD(KEY_BASE,0)&0x03) == 0x02){

    	   current_focus = Focus_Window(320,240);
       }
	   // touch KEY1 to ZOOM
	         if((IORD(KEY_BASE,0)&0x03) == 0x01){
	      	   if(bin_level == 3 )bin_level = 1;
	      	   else bin_level ++;
	      	   printf("set bin level to %d\n",bin_level);
	      	   MIPI_BIN_LEVEL(bin_level);
	      	 	usleep(500000);

	         }


	#if 0
       if((IORD(KEY_BASE,0)&0x0F) == 0x0E){

    	   current_focus = Focus_Window(320,240);
       }

       // touch KEY1 to trigger Manual focus  - step
       if((IORD(KEY_BASE,0)&0x0F) == 0x0D){

    	   if(current_focus > manual_focus_step) current_focus -= manual_focus_step;
    	   else current_focus = 0;
    	   OV8865_FOCUS_Move_to(current_focus);

       }

       // touch KEY2 to trigger Manual focus  + step
       if((IORD(KEY_BASE,0)&0x0F) == 0x0B){
    	   current_focus += manual_focus_step;
    	   if(current_focus >1023) current_focus = 1023;
    	   OV8865_FOCUS_Move_to(current_focus);
       }

       // touch KEY3 to ZOOM
       if((IORD(KEY_BASE,0)&0x0F) == 0x07){
    	   if(bin_level == 3 )bin_level = 1;
    	   else bin_level ++;
    	   printf("set bin level to %d\n",bin_level);
    	   MIPI_BIN_LEVEL(bin_level);
    	 	usleep(500000);

       }
	#endif

       //Read messages from the image processor and print them on the terminal
              while ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) { 	//Find out if there are words to read
            	  int word = IORD(0x42000,EEE_IMGPROC_MSG);
            	  printf("%08x ", word);
            	  if((word == 1) || (word == 4) || (word == 7) || (word == 10) || (word == 13)){
            		  //red, yellow, blue, pink, teal
            		  //word in buffer is a valid colour identifier
            		  //get the colour and coord data from the buffer
            		  int colour = word;
            		  int top_left = IORD(0x42000,EEE_IMGPROC_MSG);
            		  int bottom_right = IORD(0x42000,EEE_IMGPROC_MSG);
            		  //printf("%08x, %08x, %08x \n", x1, x2, colour);
            		  char* to_print;

            		  //extract coords from buffer data
            		  //https://stackoverflow.com/a/32819876
            		  alt_u16 x1 = top_left>>16;
            		  alt_u16 y2 = top_left;
            		  alt_u16 x2 = bottom_right>>16;
            		  alt_u16 y1 = bottom_right;

            		  //calculate metrics
            		  alt_u16 width = x2 - x1;
            		  //alt_u16 height = y2 - y1;

            		  //y=-8/7x+808/7
            		  float distance = (float)(-1.1429) * (float)width + (float)(115.4286);
            		  //distance = round(distance, 2);
            		  //printf("distance: %f\n", distance);
            		 if(distance < 20 || distance > 80){
            			 //data not usable
            			 continue;
            		 }

            		  int midpoint = (x1 + x2)/2;
            		  //printf("midpoint: %d\n", midpoint);
            		  float degrees = (float)midpoint * (float)(0.0781) - 25;
            		  //printf("degrees: %f\n", degrees);

            		  //printf("x1: %d, x2: %d, y1: %d, y2: %d \n width: %d, midpoint: %d \n distance: %d angle: %d \n", x1, x2, y1, y2, width, midpoint, (int)distance, (int)degrees);

            		  int length0 = snprintf(NULL, 0, "%d", (int)colour);
            		  int length1 = snprintf(NULL, 0, "%d", (int)distance);
            		  int length2 = snprintf(NULL, 0, "%d", (int)degrees);
            		  char* str0 = malloc(length0 + 1);
            		  char* str1 = malloc(length1 + 1);
            		  char* str2 = malloc(length2 + 1);
            		  snprintf(str0, length0 + 1, "%d", (int)colour);
            		  snprintf(str1, length1 + 1, "%d", (int)distance);
            		  snprintf(str2, length2 + 1, "%d", (int)degrees);

            		  //convert all of the data to a string to be passed through UART
            		  //https://stackoverflow.com/a/308712
            		  strcpy(to_print, "/[");
            		  strcat(to_print, str0);
            		  strcat(to_print, "][");
            		  strcat(to_print, str1);
            		  strcat(to_print, "][");
            		  strcat(to_print, str2);
            		  strcat(to_print, "]/");
            		  //strcat(to_print, "\0");
            		  //printf("%s \n", to_print);

            		  //establish UART connection and print output
            		  FILE* fp;
            		  fp = fopen("/dev/uart_0", "r+");
            		  if(fp){
            			  fprintf(fp, to_print);
            			  printf("Sent on UART: %s \n", to_print);
            			  usleep(10000);
            		  }else{
            			  printf("Failed to open UART\n");
            		  }

            		  //cleanup
            		  fclose(fp);
            		  free(str1);
            		  free(str2);
            	  }else{
            		  //word in buffer is not a valid colour identifier
            		  printf("invalid colour ident: %08x \n", word);
            	  }
              }

//                     		FILE* fp;
//
//                     		int i = 1;
//                     		char* send;
//                     		while(1){
//                     			printf("Try #%i\n", i);
//                     			fp = fopen("/dev/uart_0", "r+");
//                     			if(fp){
//                     			    printf("Opened UART\n");
//                     			    if(i % 2){
//                     			    	send = "a";
//                     			    }else{
//                     			    	send = "b";
//                     			    }
//                     			    fprintf(fp, send);
//                     			    printf("\'%s\' sent\n", send);
//                     			} else {
//                     			    printf("Failed to open UART\n");
//                     			}
//                     			fclose(fp);
//                     			i++;
//                     			usleep(1000000);
//                     		}

       //Update the bounding box colour
       boundingBoxColour = ((boundingBoxColour + 1) & 0xff);
       IOWR(0x42000, EEE_IMGPROC_BBCOL, (boundingBoxColour << 8) | (0xff - boundingBoxColour));

       //Process input commands
       int in = getchar();
       switch (in) {
       	   case 'e': {
       		   exposureTime += EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 'd': {
       		   exposureTime -= EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 't': {
       		   gain += GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'g': {
       		   gain -= GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'r': {
        	   current_focus += manual_focus_step;
        	   if(current_focus >1023) current_focus = 1023;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       	   case 'f': {
        	   if(current_focus > manual_focus_step) current_focus -= manual_focus_step;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       }

       		//printf("Hello\n");


//       	   	  FILE* fp;
//       	   	  int j = 0;
//       	   	  while(1){
//       	   		  j++;
//       	   		  printf("Try No. %i\n", j);
//       	   		  fp = fopen("/dev/uart_0", "r+");
//       	   		  if(fp){
//       	   			  while(1){
//       	   				  fprintf(fp, "test");
//       	   				  printf("Sent test\n");
//       	   			  }
//       	   			  fclose(fp);
//       	   		  }else{
//       	   			  printf("Unable to connect to UART\n");
//       	   		  }
//       	   		  fclose(fp);
//       	   	  }

//       	    while ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0XFF) {
//       	    	int word = IORD(0x42000, EEE_IMGPROC_MSG);
//       	    	if(fwrite(&word, 4, 1, ser) != 1){
//       	    		printf("Error writing to UART");
//       	    	}
//       	    	if(word == EEE_IMGPROC_MSG_START){
//       	    		printf("\n");
//       	    	}
//       	    	printf("%08x ", word);
//       	    }


	   //Main loop delay
	   usleep(10000);

   };
  return 0;
}
