/*
  Name: SaabOpenProg
  Copyright: Freeware, use as you please
  Author: Tomi Liljemark (firstname.surname@gmail.com)
  Created: 2007-07-16
  Modified: 2007-10-22
  Compiler: Dev-C++ v4.9.9.2 (shouldn't matter)
            Uses libraries canusbdrv.lib and FTD2XX.lib
  Description: Uses Saab I-Bus to read/program Saab Trionic 7 ECU
*/

#include <stdio.h>
#include <stdlib.h>
#include "lawcel_canusb_ftd2xx.h"

#define RELEASE_VERSION "0.88"
#define RELEASE_DATE    "2007-10-22"

#define READ            0x01
#define WRITE           0x02
#define RAW_WRITE       0x04
#define TIS_WRITE       0x08
#define VERIFY          0x80

#define ESC   27

#define CANUSB_ACCEPTANCE_CODE_LIGHT	0xFF5FFF5F
#define CANUSB_ACCEPTANCE_MASK_LIGHT	0xFF1FFF1F

/* function prototypes */
int load_file(const char *filename, unsigned char *data);
int save_file(const char *filename, const unsigned char *data);
int send_msg( CANHANDLE handle, int id, const unsigned char *data );
void ask_header( CANHANDLE handle, unsigned char header_id, unsigned char *answer);
void ask_header2( CANHANDLE handle, unsigned char header_id, unsigned char *answer);
int wait_for_msg( CANHANDLE handle, int id, int timeout, unsigned char *data );
int authenticate( CANHANDLE handle );
int erase_trionic( CANHANDLE handle );
int program_trionic( CANHANDLE handle, unsigned char *bin, const char *vin, const char *swdate, const char *tester );
int program_trionic_tis( CANHANDLE handle, unsigned char *bin, const char *vin, const char *swdate, const char *tester );
int read_trionic( CANHANDLE handle, int addr, int len, unsigned char *bin);
int verify_trionic( CANHANDLE handle, int addr, int len, const unsigned char *written);
int write_data_block( CANHANDLE handle, unsigned char header_id, const unsigned char *block);
unsigned short calc_auth_key( unsigned short seed, unsigned char method );
int get_header_field_string(const unsigned char *bin, unsigned char id, unsigned char *answer);
int strip_header_field(unsigned char *bin);
int verify_binary( const unsigned char *written, const unsigned char *read );

long gettickscount();

/* global constants */
const char init_msg[8]     = { 0x3F, 0x81, 0x00, 0x11, 0x02, 0x40, 0x00, 0x00 };

/* global variables */
unsigned char binary[512*1024];         /* Store to whole 512 kB binary in RAM */
unsigned char read_binary[512*1024];    /* Store to whole 512 kB binary in RAM */
FILE *log_output;
int binary_length = 0;


int main(int argc, char *argv[])
{
    //CANHANDLE h;
	FT_HANDLE h = NULL;
    CANMsg msg;
	UCHAR pucLatency;
    FILE *bin;
    int ch, ret, i, k, j;
    //int timestamp, last_timestamp;
    unsigned char data[8], buf[256], vin[18], swdate[7], tester[14], immo[16];
    LPTSTR verinfo;
    unsigned short seed, key;
    DWORD dwStart, dwLength;
    //HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    char operation;
	FT_STATUS ftStatus;
    
    //SetConsoleTitle("SaabOpenProg v" RELEASE_VERSION );

    printf("SaabOpenProg v%s - Read/Program Saab Trionic 7 ECU with Lawicel CANUSB\n"
           "by Tomi Liljemark %s\n\n", RELEASE_VERSION, RELEASE_DATE);


    if( argc < 3 )
    {
        printf("Usage: SaabOpenProg <R|W|A> [V] <filename.bin>\n\n"
               "Where R = Read from Trionic to PC\n"
               "      W = Write from PC to Trionic\n"
               "      A = Raw write from PC to Trionic\n"
               "      T = Write \"TIS\" binary from PC to Trionic\n"
               "      V = Verify written data (not implemented yet!)\n");
        return -1;
    }

    if( *argv[1] == 'W' || *argv[1] == 'w' )
        operation = WRITE;
    else if ( *argv[1] == 'A' || *argv[1] == 'a' )
        operation = WRITE | RAW_WRITE;
    else if ( *argv[1] == 'T' || *argv[1] == 't' )
        operation = WRITE | TIS_WRITE;
    else if( *argv[1] == 'R' || *argv[1] == 'r' )
        operation = READ;
    
    // Change the extension of the filename to .log
    // and open file for writing debug info
    strncpy( buf, argv[argc-1], 255 );
    i = strlen( buf );
    while( i > 0 )
    {
        if( buf[i-1] == '.' )
        {
            buf[i]   = 'l';
            buf[i+1] = 'o';
            buf[i+2] = 'g';
            buf[i+3] = 0;
            break;
        }
        i--;
    }
    if( i == 0 ) strcat( buf, ".log" );
    log_output = fopen( buf, "w" );    

    fprintf( log_output, "SaabOpenProg v%s - Read/Program Saab Trionic 7 ECU with Lawicel CANUSB\n"
                         "by Tomi Liljemark %s\n\n", RELEASE_VERSION, RELEASE_DATE);


    if( operation & WRITE )    
    {
        if ( argc > 3 )
        {
            if( *argv[2] == 'V' || *argv[2] == 'v' ) operation |= VERIFY;
        }
        if( load_file( argv[argc-1], binary ) )
        {
            printf("Error: could not load file %s!\n", argv[argc-1]);
            fprintf( log_output, "Error: could not load file %s!\n", argv[argc-1]);
            fclose(log_output);
            return -1;
        }

        // Check that the file begins with FF FF EF FC
        if( binary[0] != 0xFF || binary[1] != 0xFF || binary[2] != 0xEF || binary[3] != 0xFC )
        {
            printf("Error: binary doesn't appear to be for a Trionic 7 ECU! (%02X%02X%02X%02X)\n", 
                binary[0], binary[1], binary[2], binary[3] );
            fprintf( log_output, "Error: binary doesn't appear to be for a Trionic 7 ECU! (%02X%02X%02X%02X)\n", 
                binary[0], binary[1], binary[2], binary[3] );
            fclose(log_output);
            return -1;
        }

        if( operation & TIS_WRITE )
        {
            printf("\nInformation read from the \"TIS\" binary\n"
                     "--------------------------------------\n");
            fprintf( log_output, "\nInformation read from the \"TIS\" binary\n"
                     "--------------------------------\n");
            if( !get_header_field_string( binary, 0x91, buf ) ) strcpy( buf, "N/A" );
            printf("Box HW part number   : %s\n", buf);
            fprintf( log_output, "Box HW part number   : %s\n", buf);
            if( !get_header_field_string( binary, 0x94, buf ) ) strcpy( buf, "N/A" );
            printf("Box SW part number   : %s\n", buf);
            fprintf( log_output, "Box SW part number   : %s\n", buf);
            if( !get_header_field_string( binary, 0x95, buf ) ) strcpy( buf, "N/A" );
            printf("ECU Software version : %s\n", buf);
            fprintf( log_output, "ECU Software version : %s\n", buf);
            if( !get_header_field_string( binary, 0x97, buf ) ) strcpy( buf, "N/A" );
            printf("Engine type          : %s\n\n", buf);
            fprintf( log_output, "Engine type          : %s\n\n", buf);
        }
        else if( get_header_field_string( binary, 0x90, vin ) &&
            get_header_field_string( binary, 0x99, swdate ) &&
            get_header_field_string( binary, 0x98, tester ) && 
            get_header_field_string( binary, 0x92, immo ) )
        {
            printf("\nInformation read from the binary\n"
                     "--------------------------------\n");
            fprintf( log_output, "\nInformation read from the binary\n"
                     "--------------------------------\n");
            printf("VIN                  : %s\n", vin);
            fprintf( log_output, "VIN                  : %s\n", vin);
            if( !get_header_field_string( binary, 0x91, buf ) ) strcpy( buf, "N/A" );
            printf("Box HW part number   : %s\n", buf);
            fprintf( log_output, "Box HW part number   : %s\n", buf);
            if( !get_header_field_string( binary, 0x94, buf ) ) strcpy( buf, "N/A" );
            printf("Box SW part number   : %s\n", buf);
            fprintf( log_output, "Box SW part number   : %s\n", buf);
            if( !get_header_field_string( binary, 0x95, buf ) ) strcpy( buf, "N/A" );
            printf("ECU Software version : %s\n", buf);
            fprintf( log_output, "ECU Software version : %s\n", buf);
            if( !get_header_field_string( binary, 0x97, buf ) ) strcpy( buf, "N/A" );
            printf("Engine type          : %s\n", buf);
            fprintf( log_output, "Engine type          : %s\n", buf);
            printf("Hardware serial nr   : %s\n", immo);
            fprintf( log_output, "Hardware serial nr   : %s\n", immo);
            printf("Tester info          : %s\n", tester);
            fprintf( log_output, "Tester info          : %s\n", tester);
            printf("Software date        : %s\n\n", swdate);
            fprintf( log_output, "Software date        : %s\n\n", swdate);

            if( !(operation & RAW_WRITE) )
            {
                if( !strip_header_field( binary ) )
                {
                    printf("Error: failed to remove header fields - internal program error?\n");
                    fprintf( log_output, "Error: failed to remove header fields - internal program error?\n");
                    fclose(log_output);
                    return -1;
                }
            }
        }
        else
        {
            printf("Error: failed to read header information from binary!\n");
            fprintf( log_output, "Error: failed to read header information from binary!\n");
            fclose(log_output);
            return -1;
        }
    }
    else if( operation & READ )
    {
        bin = fopen( argv[argc-1], "wb" );
        if( bin == NULL )
        {
            printf("Error: could not open file %s!\n", argv[argc-1]);
            fprintf( log_output, "Error: could not open file %s!\n", argv[argc-1]);
            fclose(log_output);
            return -1;
        }
        else
        {
            fclose(bin);
        }
    }
    else
    {
        printf("Usage: SaabOpenProg <R|W|A> [V] <filename.bin>\n\n"
               "Where R = Read from Trionic to PC\n"
               "      W = Write from PC to Trionic\n"
               "      A = Raw write from PC to Trionic\n"
               "      V = Verify written data (not implemented yet!)\n");
        fclose(log_output);
        return -1;
    }
    

	initializeCanUsb();

    printf("Opening CAN channel to Saab I-Bus (47,619 kBit/s)...");
    fprintf( log_output, "Opening CAN channel to Saab I-Bus (47,619 kBit/s)...");
    // Open CAN Channel
    //if ( 0 >= ( h = canusb_Open( NULL,
    //                            "0xcb:0x9a",
    //                            CANUSB_ACCEPTANCE_CODE_LIGHT,
    //                            CANUSB_ACCEPTANCE_MASK_LIGHT,
    //                            CANUSB_FLAG_TIMESTAMP ) ) ) {
	ftStatus = FT_OpenEx( "CANUSB", FT_OPEN_BY_DESCRIPTION, &h);
	if(ftStatus != FT_OK) {
		printf("FT_OpenEx() failed. rv=%d\n", ftStatus);
		printf("Failed to open device\n");
        fprintf( log_output, "Failed to open device\n");
        fclose(log_output);
		return -1;
	}
	
	FT_ResetDevice(&h);
	FT_Purge(&h, 3); // rx+tx
	
	
	setTimeouts( h, 0x80, 0x17A );       // 3 second read + write timeouts
	FT_SetUSBParameters(&h, 0x8000, 0);
	
	FT_GetLatencyTimer(&h, &pucLatency); //25 ms
	printf("getlatency=%u\n",pucLatency);
	pucLatency=10;
	FT_SetLatencyTimer(&h, pucLatency);
	FT_GetLatencyTimer(&h, &pucLatency); //25 ms
	printf("getlatency=%u\n",pucLatency);
	//setTimeStampOn(h);
	setCodeRegister(h);
	setMaskRegister(h);
	
	//bitrates:
	//  "scb9a" == (47,619kBits/s (0xcb:0x9a)
	//"S6" == 500kbit/s
	if ( !openChannel( h, "S6\r" ) ) { //scb9a\r
		printf("Failed to open channel\n");
		return FALSE;
	}
	printf("OK channel open\n");
  
    printf("ok\n");
    fprintf( log_output, "ok\n");
    

    if( wait_for_msg( h, 0, 250, data ) != 0 )
    {
        printf("Message received from bus, everything seems OK.\n");
        fprintf( log_output, "Message received from bus, everything seems OK.\n");
    }
    else
    {
        // Flush data CAN channel
        //canusb_Flush( h, FLUSH_WAIT );
		FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
        
        // Close CAN channel
        closeChannel( h );

        printf("Opening CAN channel to Saab P-Bus (500 kBit/s)...");
        fprintf( log_output, "Opening CAN channel to Saab P-Bus (500 kBit/s)...");
        // Open CAN Channel
        //if ( 0 >= ( h = canusb_Open( NULL,
        //                            "500",
        //                            CANUSB_ACCEPTANCE_CODE_LIGHT,
        //                            CANUSB_ACCEPTANCE_MASK_LIGHT,
        //                            CANUSB_FLAG_TIMESTAMP ) ) ) {
		//"S6" == 500kbit/s
		if ( !openChannel( h, "S6\r" ) ) {
			printf("Failed to open device\n");
            fprintf( log_output, "Failed to open device\n");
            fclose(log_output);
            return -1;
        }
        printf("ok\n");
        fprintf( log_output, "ok\n");

        if( wait_for_msg( h, 0, 250, data ) != 0 )
        {
            printf("Message received from bus, everything seems OK.\n");
            fprintf( log_output, "Message received from bus, everything seems OK.\n");
        }
        else
        {
            printf("Error: could not receive any messages from either I-Bus or P-Bus!\n");
            fprintf( log_output, "Error: could not receive any messages from either I-Bus or P-Bus!\n");
            
            // Flush data CAN channel
            //canusb_Flush( h, FLUSH_WAIT );
			FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
            
            // Close CAN channel
            closeChannel( h );
            printf("\nCAN channel closed.\n");
            fprintf( log_output, "\nCAN channel closed.\n");
            fclose(log_output);
            return -1;
        }
    }


    // Acquire Trionic information
    printf("Initialization...");
    fprintf( log_output, "Initialization...");
    ret = send_msg( h, 0x220, init_msg );
    if( ret == ERROR_CANUSB_OK )
    {
        if( wait_for_msg( h, 0x238, 1000, data ) == 0x238 )
        {
            printf("ok\n");
            fprintf( log_output, "ok\n");
/* DEBUG info...
            for( i = 0; i < 8; i++ ) printf("0x%02X ", data[i] );
            printf("\n");
*/
        }
        else 
        {
            printf("failed\n");
            fprintf( log_output, "failed\n");
        }
        
        printf("\nInformation requested from the Trionic\n"
                 "--------------------------------------\n");
        fprintf( log_output, "\nInformation requested from the Trionic\n"
                             "--------------------------------------\n");
        
        buf[0] = 0x00;
        ask_header( h, 0x90, buf );
        if( buf[0] != 0x00 )
        {
            printf("VIN                  : %s\n", buf);
            fprintf( log_output, "VIN                  : %s\n", buf);
        }
        if( operation & TIS_WRITE ) strncpy( vin, buf, sizeof(vin) );

        buf[0] = 0x00;
        ask_header( h, 0x91, buf );
        if( buf[0] != 0x00 )
        {
            printf("Box HW part number   : %s\n", buf);
            fprintf( log_output, "Box HW part number   : %s\n", buf);
        }
    
        buf[0] = 0x00;
        ask_header( h, 0x94, buf );
        if( buf[0] != 0x00 )
        {
            printf("Box SW part number   : %s\n", buf);
            fprintf( log_output, "Box SW part number   : %s\n", buf);
        }
    
        buf[0] = 0x00;
        ask_header( h, 0x95, buf );
        if( buf[0] != 0x00 )
        {
            printf("ECU Software version : %s\n", buf);
            fprintf( log_output, "ECU Software version : %s\n", buf);
        }
        
        buf[0] = 0x00;
        ask_header( h, 0x97, buf );
        if( buf[0] != 0x00 )
        {
            printf("Engine type          : %s\n", buf);
            fprintf( log_output, "Engine type          : %s\n", buf);
        }

        buf[0] = 0x00;
        ask_header( h, 0x92, buf );
        if( buf[0] != 0x00 )
        {
            printf("Hardware serial nr   : %s\n", buf);
            fprintf( log_output, "Hardware serial nr   : %s\n", buf);
        }

        buf[0] = 0x00;
        ask_header( h, 0x98, buf );
        if( buf[0] != 0x00 )
        {
            printf("Tester info          : %s\n", buf);
            fprintf( log_output, "Tester info          : %s\n", buf);
        }
        if( operation & TIS_WRITE ) strncpy( tester, buf, sizeof(tester) );

        buf[0] = 0x00;
        ask_header( h, 0x99, buf );
        if( buf[0] != 0x00 )
        {
            printf("Software date        : %s\n\n", buf);
            fprintf( log_output, "Software date        : %s\n\n", buf);
        }
        if( operation & TIS_WRITE ) strncpy( swdate, buf, sizeof(swdate) );

    }
    else
    {
        printf("Send failed, err %d.\n", ret);
        fprintf( log_output, "Send failed, err %d.\n", ret);
        
    }

    if( operation & WRITE )
    {
        // Lets use our own tester text... remove this to use the one in the binary
        // In case user wants Raw write, respect the one in the binary...
        if( !(operation & RAW_WRITE) ) strncpy( tester, "SAAB_OPEN_PRG", 13 );
    
        // Confirm that the user really wants to program
        dwStart = gettickscount();
        printf(/*"Ensure that the VIN shown above is the correct one!\n\n"*/
               "Note! If programming fails, you will probably have to re-program it using the\n"
               "BDM interface. This means getting the right hardware, opening the Trionic box,\n"
               "soldering a pin header to the circuit board and using special software.\n\n");
    
        printf("Are you SURE you want to program [y/N] ? ");
        buf[0] = (unsigned char)getchar();
        printf("\n");    
        if( buf[0] != 'y' && buf[0] != 'Y' )
        {
            printf("Aborted, nothing done.\n");
            fprintf( log_output, "Aborted, nothing done.\n");
            fclose(log_output);
            return 0;
        }
    }    

    // Authenticate
    printf("Authentication...");
    fprintf( log_output, "Authentication...");
    
    if( authenticate( h ) == 0 )
    {
        printf("ok\n");
        fprintf( log_output, "ok\n");
    }
    else
    {
        printf("failed\n");
        fprintf( log_output, "failed\n");
        // Flush data CAN channel
        //canusb_Flush( h, FLUSH_WAIT );
		FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
        
        // Close CAN channel
        closeChannel( h );
        printf("\nCAN channel closed.\n");
        fprintf( log_output, "\nCAN channel closed.\n");
        fclose(log_output);
        return -1;
    }

    if( operation & WRITE )
    {
        // Erase
        printf("Erase...");
        fprintf( log_output, "Erase...");
        dwStart = gettickscount();
        if( erase_trionic( h ) == 0 )
        {
            dwLength = gettickscount() - dwStart;
            printf("ok (%3.1f s)\n", (float)dwLength/1000.0);
            fprintf( log_output, "ok (%3.1f s)\n", (float)dwLength/1000.0);
        }
        else
        {
            dwLength = gettickscount() - dwStart;
            printf("failed (%3.1f s)\n", (float)dwLength/1000.0);
            fprintf( log_output, "failed (%3.1f s)\n", (float)dwLength/1000.0);
            // Flush data CAN channel
            //canusb_Flush( h, FLUSH_WAIT );
			FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
            
            // Close CAN channel
            closeChannel( h );
            printf("\nCAN channel closed.\n");
            fprintf( log_output, "\nCAN channel closed.\n");
            fclose(log_output);
            return -1;
        }
    
    
        // Program
        if( operation & RAW_WRITE )
        {
            printf("Programming (Raw mode)...");
            fprintf( log_output, "Programming (Raw mode)...");
            dwStart = gettickscount();
            i = program_trionic( h, binary, NULL, NULL, NULL );
        }
        else if( operation & TIS_WRITE )
        {
            printf("Programming (TIS mode)...");
            fprintf( log_output, "Programming (TIS mode)...");
            dwStart = gettickscount();
            i = program_trionic_tis( h, binary, vin, swdate, tester );
        }
        else
        {
            printf("Programming...");
            fprintf( log_output, "Programming...");
            dwStart = gettickscount();
            i = program_trionic( h, binary, vin, swdate, tester );
        }

        // Was the programming a success?
        if( i == 0 )
        {
            dwLength = gettickscount() - dwStart;
            printf(" - ok (%4.1f min)\n", (float)dwLength/60000.0);
            fprintf( log_output, " - ok (%4.1f min)\n", (float)dwLength/60000.0);
        }
        else
        {
            dwLength = gettickscount() - dwStart;
            printf(" - failed (%4.1f min)\n", (float)dwLength/60000.0);
            fprintf( log_output, " - failed (%4.1f min)\n", (float)dwLength/60000.0);
            // Flush data CAN channel
            //canusb_Flush( h, FLUSH_WAIT );
			FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
            
            // Close CAN channel
            closeChannel( h );
            printf("\nCAN channel closed.\n");
            fprintf( log_output, "\nCAN channel closed.\n");
            fclose(log_output);
            return -1;
        }
    }

    if( operation & READ )
    {
        // Read
        printf("Reading..." );
        fprintf( log_output, "Reading..." );
        dwStart = gettickscount();
        i = read_trionic( h, 0x0, 0x80000, read_binary );
        dwLength = gettickscount() - dwStart;
    
        if( i == 0x80000 )
        {
            printf(" - ok (%4.1f min)\n", (float)dwLength/60000.0);
            fprintf( log_output, " - ok (%4.1f min)\n", (float)dwLength/60000.0);
        }
        else
        {
            printf(" - failed (%4.1f min)\n", (float)dwLength/60000.0);
            fprintf( log_output, " - failed (%4.1f min)\n", (float)dwLength/60000.0);
        }
    
        if( save_file( argv[argc-1], read_binary ) != 0 )
        {
            printf("Error: write failed!\n");
            fprintf( log_output, "Error: write failed!\n");
        }
    }
    /* NOT READY YET...
    else if( operation & VERIFY )
    {
        // Verify-after-write
        printf("Verifying...");
        dwStart = GetTickCount();
        i = verify_trionic( h, 0x0, 0x80000, binary );
        dwLength = GetTickCount() - dwStart;
    
        if( i == 0 )
        {
            printf(" - ok (%4.1f min)\n", (float)dwLength/60000.0);
        }
        else
        {
            printf(" - failed (%4.1f min)\n", (float)dwLength/60000.0);
        }
    }
    */
    
    // Flush data CAN channel
    //canusb_Flush( h, FLUSH_WAIT );
	FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
    
    // Close CAN channel
    closeChannel( h );
    printf("\nCAN channel closed.\n");
    fprintf( log_output, "\nCAN channel closed.\n");

    fclose(log_output);
    return 0;
}

int load_file(const char *filename, unsigned char *data)
{
    FILE *bin;
    int i;
    size_t read_bytes;
    unsigned char temp;

    read_bytes = 0;
    bin = fopen( filename, "rb" );

    if( bin != NULL )
    {    
        fseek( bin, 0, SEEK_SET );
        read_bytes = fread( data, sizeof(unsigned char), 512*1024, bin );
        fclose( bin );
    }
    else
    {
        printf("Error: could not open file %s!\n", filename);
        fprintf( log_output, "Error: could not open file %s!\n", filename);
    }
    
    if( read_bytes == 256*1024 )
    {
        printf("Error: is this a Trionic 5 ECU binary?\n");
        fprintf( log_output, "Error: is this a Trionic 5 ECU binary?\n");
    }
    else if( read_bytes == 512*1024 || read_bytes == 0x70100 )
    {
        // Convert Motorola byte-order to Intel byte-order (just in RAM)
        if( data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFC && data[3] == 0xEF )
        {
            for( i = 0; i < read_bytes; i += 2 )
            {
                temp = data[i];
                data[i] = data[i+1];
                data[i+1] = temp;
            }
            printf("Note: Motorola byte-order detected.\n");
            fprintf( log_output, "Note: Motorola byte-order detected.\n");
        }
        
    }

    binary_length = read_bytes;
    return (read_bytes == 512*1024 || read_bytes == 0x70100 ) ? 0 : -1;
}

int save_file(const char *filename, const unsigned char *data)
{
    FILE *bin;
    int i;
    size_t write_bytes;

    write_bytes = 0;
    bin = fopen( filename, "wb" );

    if( bin != NULL )
    {    
        fseek( bin, 0, SEEK_SET );
        write_bytes = fwrite( data, sizeof(unsigned char), 512*1024, bin );
        fclose( bin );
    }
    else
    {
        printf("Error: could not open file %s!\n", filename);
        fprintf( log_output, "Error: could not open file %s!\n", filename);
    }
    
    return write_bytes == 512*1024 ? 0 : -1;
}

unsigned short calc_auth_key( unsigned short seed, unsigned char method )
{
    unsigned short key;
    
    key = seed << 2;
    key &= 0xFFFF;
    key ^= ( method ? 0x4081 : 0x8142 );
    key -= ( method ? 0x1F6F : 0x2356 );
    key &= 0xFFFF;
    
    return key;
}

void ask_header( CANHANDLE handle, unsigned char header_id, unsigned char *answer)
{
    unsigned char data[8], length, i;
    int ret;
    unsigned char query[8] = { 0x40, 0xA1, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00 };
    unsigned char ack[8]   = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    
   
    query[4] = header_id;
    data[0] = 0x00;
    
    // Send query to Trionic
    ret = send_msg( handle, 0x240, query );
    
    if( ret != ERROR_CANUSB_OK )
    {
        printf("Send failed, err %d.\n", ret);
        fprintf( log_output, "Send failed, err %d.\n", ret);
        usleep(100000); //sleep(100);
        // Send query to Trionic
        ret = send_msg( handle, 0x240, query );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send retry failed, err %d.\n", ret);
            fprintf( log_output, "Send retry failed, err %d.\n", ret);
        }
    }

    // Read response messages
    while( data[0] != 0x80 && data[0] != 0xC0 )
    {
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            if( data[0] & 0x40 )
            {
                if( data[2] > 2 ) length = data[2] - 2;   // subtract two non-payload bytes
                else length = 0;
                if( --length > 0 ) *answer++ = data[5];
                if( --length > 0 ) *answer++ = data[6];
                if( --length > 0 ) *answer++ = data[7];
            }
            else
            {
                for( i = 0; i < 6; i++ )
                {
                    *answer++ = data[2+i];
                    length--;
                    if( length == 0 ) i = 6;
                }
            }
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            ret = send_msg( handle, 0x266, ack );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'ack' failed, err %d.\n", ret);
                fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
				usleep(100000); //sleep(100);
                // Send query to Trionic
                ret = send_msg( handle, 0x266, ack );
                if( ret != ERROR_CANUSB_OK )
                {
                    printf("Send 'ack' retry failed, err %d.\n", ret);
                    fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
                }
            }
        }
        else
        {
            // Timeout
            printf("Timeout waiting for 0x258.\n");
            fprintf( log_output, "Timeout waiting for 0x258.\n");
            return;
        }
    }

    // Set end of string
    *answer = 0;
}

int authenticate( CANHANDLE handle )
{
    unsigned char data[8], i;
    int ret;
    const char security_msg[8] = { 0x40, 0xA1, 0x02, 0x27, 0x05, 0x00, 0x00, 0x00 };
    char security_msg_reply[8] = { 0x40, 0xA1, 0x04, 0x27, 0x06, 0x00, 0x00, 0x00 };
    unsigned char ack[8]       = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned short seed, key;
    
   
    // Send "Request Seed" to Trionic
    ret = send_msg( handle, 0x240, security_msg );
    if( ret != ERROR_CANUSB_OK )
    {
        printf("Send 'security_msg' failed, err %d.\n", ret);
        fprintf( log_output, "Send 'security_msg' failed, err %d.\n", ret);

        usleep(100000); //sleep(100);
        // Retry
        ret = send_msg( handle, 0x240, security_msg );
        
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'security_msg' retry failed, err %d.\n", ret);
            fprintf( log_output, "Send 'security_msg' retry failed, err %d.\n", ret);
        }
    }

    // Read "Seed"
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        ret = send_msg( handle, 0x266, ack );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'ack' failed, err %d.\n", ret);
            fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
            usleep(100000); //sleep(100);
            // Retry acknowledgement
            ret = send_msg( handle, 0x266, ack );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'ack' retry failed, err %d.\n", ret);
                fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
            }
        }

        // Send "Key"
        seed = data[5] << 8 | data[6];
        key = calc_auth_key( seed, 0 );
        security_msg_reply[5] = ( key >> 8 ) & 0xFF;
        security_msg_reply[6] = key & 0xFF;
        ret = send_msg( handle, 0x240, security_msg_reply );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'security_msg_reply' failed, err %d.\n", ret);
            fprintf( log_output, "Send 'security_msg_reply' failed, err %d.\n", ret);
            usleep(100000); //sleep(100);
            // Retry
            ret = send_msg( handle, 0x240, security_msg_reply );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'security_msg_reply' retry failed, err %d.\n", ret);
                fprintf( log_output, "Send 'security_msg_reply' retry failed, err %d.\n", ret);
            }
        }
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            ret = send_msg( handle, 0x266, ack );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'ack' failed, err %d.\n", ret);
                fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
                usleep(100000); //sleep(100);
                // Retry
                ret = send_msg( handle, 0x266, ack );
                if( ret != ERROR_CANUSB_OK )
                {
                    printf("Send 'ack' retry failed, err %d.\n", ret);
                    fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
                }
            }

            if( data[3] == 0x67 && data[5] == 0x34 )
            {
                // "Key" accepted!
                return 0;
            }
            else
            {
                // Send "Key" (second try)
                key = calc_auth_key( seed, 1 );
                security_msg_reply[5] = ( key >> 8 ) & 0xFF;
                security_msg_reply[6] = key & 0xFF;
                ret = send_msg( handle, 0x240, security_msg_reply );
                if( ret != ERROR_CANUSB_OK )
                {
                    printf("Send 'security_msg_reply' failed, err %d.\n", ret);
                    fprintf( log_output, "Send 'security_msg_reply' failed, err %d.\n", ret);
                    usleep(100000); //sleep(100);
                    // Retry
                    ret = send_msg( handle, 0x240, security_msg_reply );
                    if( ret != ERROR_CANUSB_OK )
                    {
                        printf("Send 'security_msg_reply' retry failed, err %d.\n", ret);
                        fprintf( log_output, "Send 'security_msg_reply' retry failed, err %d.\n", ret);
                    }
                }
                if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                {
                    // Send acknowledgement
                    ack[3] = data[0] & 0xBF;
                    ret = send_msg( handle, 0x266, ack );
                    if( ret != ERROR_CANUSB_OK )
                    {
                        printf("Send 'ack' failed, err %d.\n", ret);
                        fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
                        usleep(100000); //sleep(100);
                        // Retry
                        ret = send_msg( handle, 0x266, ack );
                        if( ret != ERROR_CANUSB_OK )
                        {
                            printf("Send 'ack' retry failed, err %d.\n", ret);
                            fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
                        }
                    }
                    if( data[3] == 0x67 && data[5] == 0x34 )
                    {
                        // "Key" accepted!
                        return 0;
                    }
                }
            }
        }
        
    }
    else
    {
        // Timeout
        printf("Timeout waiting for 0x258.\n");
        fprintf( log_output, "Timeout waiting for 0x258.\n");
    }
    return -1;
}

int erase_trionic( CANHANDLE handle )
{
    unsigned char data[8], i;
    const char erase_msg1[8]   = { 0x40, 0xA1, 0x02, 0x31, 0x52, 0x00, 0x00, 0x00 };
    const char erase_msg2[8]   = { 0x40, 0xA1, 0x02, 0x31, 0x53, 0x00, 0x00, 0x00 };
    const char confirm_msg[8]  = { 0x40, 0xA1, 0x01, 0x3E, 0x00, 0x00, 0x00, 0x00 };
    unsigned char ack[8]       = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    
       
    // Send "Erase message 1" to Trionic
    data[3] = 0;
    i = 0;
    while( data[3] != 0x71 && i < 10)
    {
        send_msg( handle, 0x240, erase_msg1 );

        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
        }
        else
        {
            // Timeout
            printf("Timeout waiting for response to 'Erase message 1'.\n");
            fprintf( log_output, "Timeout waiting for response to 'Erase message 1'.\n");
            return -1;
        }
        usleep(100000); //sleep( 100 );
        i++;
    }

    if( i >= 10 ) return -1;
    
    // Send "Erase message 2" to Trionic
    data[3] = 0;
    i = 0;
    while( data[3] != 0x71 && i < 200)
    {
        send_msg( handle, 0x240, erase_msg2 );

        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
        }
        else
        {
            // Timeout
            printf("Timeout waiting for response to 'Erase message 2'.\n");
            fprintf( log_output, "Timeout waiting for response to 'Erase message 2'.\n");
            return -1;
        }
        usleep(100000); //sleep( 100 );
        i++;
    }

    // Check to see if erase operation lasted longer than 20 sec...
    if( i >= 200 ) return -1;

    // Confirm erase was successful?
    // (Note: no acknowledgements used for some reason)
    send_msg( handle, 0x240, confirm_msg );

    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        if( data[3] == 0x7E )
        {
            usleep(100000); //sleep( 100 );
            send_msg( handle, 0x240, confirm_msg );
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                if( data[3] == 0x7E )
                {
                    // Everything went OK
                    return 0;
                }
            }
        }
    }

    return -1;
}

int program_trionic( CANHANDLE handle, unsigned char *bin, const char *vin, const char *swdate, const char *tester )
{
    const char jump_msg1a[8]   = { 0x41, 0xA1, 0x08, 0x34, 0x00, 0x00, 0x00, 0x00 };    // 0x000000 length=0x07B000
    const char jump_msg1b[8]   = { 0x00, 0xA1, 0x07, 0xB0, 0x00, 0x00, 0x00, 0x00 };
    const char jump_msg2a[8]   = { 0x41, 0xA1, 0x08, 0x34, 0x07, 0xFF, 0x00, 0x00 };    // 0x07FF00 length=0x000100
    const char jump_msg2b[8]   = { 0x00, 0xA1, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    const char end_data_msg[8] = { 0x40, 0xA1, 0x01, 0x37, 0x00, 0x00, 0x00, 0x00 };
    const char exit_diag_msg[8]= { 0x40, 0xA1, 0x02, 0x31, 0x54, 0x00, 0x00, 0x00 };
    const char req_diag_result_msg[8]= { 0x3F, 0x81, 0x01, 0x33, 0x02, 0x40, 0x00, 0x00 };  // 220h
    unsigned char ack[8]       = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };        // 266h
    unsigned char data[8];
    int i, k, bin_count;
    //HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    //CONSOLE_SCREEN_BUFFER_INFO csbi;
    

    bin_count = 0;
    //GetConsoleScreenBufferInfo(hout, &csbi);

    // Send "Request Download - tool to module" to Trionic
    send_msg( handle, 0x240, jump_msg1a );
    send_msg( handle, 0x240, jump_msg1b );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        
        if( data[3] == 0x74 )
        {
            // Send 0x00000...0x6FF90 (1911 message groups)
            //while( (bin_count+240) < 0x70000 )
            // Send 0x00000...0x7AFD0
            while( (bin_count+240) < 0x7B000 )
            {
                if ( bin_count % 0x147A )
                {
                    //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                    printf("%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                    //printf(".");
                }
                data[1] = 0xA1;
                for( i = 0x28; i >= 0; i-- )
                {
                    data[0] = i;
                    if( i == 0x28 )
                    {
                        data[0] |= 0x40;
                        data[2] = 0xF1; // length
                        data[3] = 0x36; // Data Transfer
                        data[4] = *(bin + bin_count++);
                        data[5] = *(bin + bin_count++);
                        data[6] = *(bin + bin_count++);
                        data[7] = *(bin + bin_count++);
                    }
                    else if ( i == 0 )
                    {
                        data[2] = *(bin + bin_count++);
                        data[3] = *(bin + bin_count++);
                        for( k = 4; k < 8; k++ ) data[k] = 0x00;
                    }
                    else
                    {
                        for( k = 2; k < 8; k++ )
                        {
                            data[k] = *(bin + bin_count++);
                        }
                    }
                    //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                    //printf("\n");
                    usleep(3000); //sleep(3);
                    send_msg( handle, 0x240, data );
                }
                // Read response
                if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                {
                    // Send acknowledgement
                    ack[3] = data[0] & 0xBF;
                    send_msg( handle, 0x266, ack );
                    if( data[3] != 0x76 )
                    {
                        // failed...
                        printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                        fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                        return -1;
                    }
                }
                else
                {
                    // failed...
                    printf("err line: %d\n", __LINE__ );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                    fprintf( log_output, "err line: %d\n", __LINE__ );
                    return -1;
                }
            }
            // Send 0x6FF91...0x70000
            // Send 0x7AFD1...0x7B000
            data[1] = 0xA1;
            //for( i = 0x12; i >= 0; i-- )
            for( i = 0x09; i >= 0; i-- )
            {
                if ( bin_count % 0x147A )
                {
                    //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                    printf("%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                    //printf("o");
                }
                data[0] = i;
                //if( i == 0x12 )
                if( i == 0x09 )
                {
                    data[0] |= 0x40;
                    //data[2] = 0x71; // length
                    data[2] = 0x30; // length
                    data[3] = 0x36; // Data Transfer
                    data[4] = *(bin + bin_count++);
                    data[5] = *(bin + bin_count++);
                    data[6] = *(bin + bin_count++);
                    data[7] = *(bin + bin_count++);
                }
                else if( i == 0 )
                {
                    data[2] = *(bin + bin_count++);
                    for( k = 3; k < 8; k++ ) data[k] = 0x00;
                }
                else
                {
                    for( k = 2; k < 8; k++ )
                    {
                        data[k] = *(bin + bin_count++);
                    }
                }
                //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                //printf("\n");
                usleep(3000); //sleep(3);
                send_msg( handle, 0x240, data );
            }
            // Read response
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                if( data[3] != 0x76 )
                {
                    // failed...
                    printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                    fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                    return -1;
                }
            }
            else
            {
                // failed...
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }

            // Send "Request Download - tool to module" to Trionic
            // (i.e. jump to address 0x7FF00)
            send_msg( handle, 0x240, jump_msg2a );
            send_msg( handle, 0x240, jump_msg2b );
            
            bin_count = 0x7FF00;
        
            // Read response
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                
                if( data[3] = 0x74 )
                {
                    // Send 0x7FF00...7FFF0
                    data[1] = 0xA1;
                    for( i = 0x28; i >= 0; i-- )
                    {
                        if ( bin_count % 0x147A )
                        {
                            //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                            printf("%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                            //printf("#");
                        }
                        data[0] = i;
                        if( i == 0x28 )
                        {
                            data[0] |= 0x40;
                            data[2] = 0xF1; // length
                            data[3] = 0x36; // Data Transfer
                            data[4] = *(bin + bin_count++);
                            data[5] = *(bin + bin_count++);
                            data[6] = *(bin + bin_count++);
                            data[7] = *(bin + bin_count++);
                        }
                        else if ( i == 0 )
                        {
                            data[2] = *(bin + bin_count++);
                            data[3] = *(bin + bin_count++);
                            for( k = 4; k < 8; k++ ) data[k] = 0x00;
                        }
                        else
                        {
                            for( k = 2; k < 8; k++ )
                            {
                                data[k] = *(bin + bin_count++);
                            }
                        }
                        //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                        //printf("\n");
                        usleep(3000); //sleep(3);
                        send_msg( handle, 0x240, data );
                    }
                    // Read response
                    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                    {
                        // Send acknowledgement
                        ack[3] = data[0] & 0xBF;
                        send_msg( handle, 0x266, ack );
                        if( data[3] != 0x76 )
                        {
                            // failed...
                            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                            return -1;
                        }
                    }
                    else
                    {
                        // failed...
                        printf("err line: %d\n", __LINE__ );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                        fprintf( log_output, "err line: %d\n", __LINE__ );
                        return -1;
                    }
                    // Send 0x7FFF0...0x7FFFFF
                    data[1] = 0xA1;
                    for( i = 0x2; i >= 0; i-- )
                    {
                        if ( bin_count % 0x147A )
                        {
                            //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                            printf("%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                            //printf("x");
                        }
                        data[0] = i;
                        if( i == 0x2 )
                        {
                            data[0] |= 0x40;
                            data[2] = 0x11; // length
                            data[3] = 0x36; // Data Transfer
                            data[4] = *(bin + bin_count++);
                            data[5] = *(bin + bin_count++);
                            data[6] = *(bin + bin_count++);
                            data[7] = *(bin + bin_count++);
                        }
                        else
                        {
                            for( k = 2; k < 8; k++ )
                            {
                                data[k] = *(bin + bin_count++);
                            }
                        }
                        //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                        //printf("\n");
                        usleep(3000); //sleep(3);
                        send_msg( handle, 0x240, data );
                    }
                    // Read response
                    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                    {
                        // Send acknowledgement
                        ack[3] = data[0] & 0xBF;
                        send_msg( handle, 0x266, ack );
                        if( data[3] != 0x76 )
                        {
                            // failed...
                            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                            return -1;
                        }
                    }
                    else
                    {
                        // failed...
                        printf("err line: %d\n", __LINE__ );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                        fprintf( log_output, "err line: %d\n", __LINE__ );
                        return -1;
                    }

                }
                else
                {
                    printf("err line: %d\n", __LINE__ );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                    fprintf( log_output, "err line: %d\n", __LINE__ );
                    return -1;
                }
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }
            

        }
        else
        {
            printf("err line: %d\n", __LINE__ );
            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
            fprintf( log_output, "err line: %d\n", __LINE__ );
            return -1;
        }
    }
    else
    {
        printf("err line: %d\n", __LINE__ );
        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(512*1024)*100.0);
        fprintf( log_output, "err line: %d\n", __LINE__ );
        return -1;
    }

    // Quit now, if Raw write has been selected
    if( vin == NULL && swdate == NULL && tester == NULL )
    {
        return 0;
    }

   // Send "Request Data Transfer Exit" to Trionic
    send_msg( handle, 0x240, end_data_msg );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        if( data[3] == 0x77 )
        {
            // Program VIN
            write_data_block( handle, 0x90, vin);
            // Program software date
            write_data_block( handle, 0x99, swdate);
            // Program tester info
            write_data_block( handle, 0x98, tester);

            // Send "Exit diagnostic routine"
            send_msg( handle, 0x240, exit_diag_msg );
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                if( data[3] != 0x71 )
                {
                    printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                    fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(512*1024)*100.0);
                    fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                    return -1;
                }             
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(512*1024)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }
/*
            // Sleep 5 seconds
            sleep(5000);
            
            // Send "Request diagnostic results"
            send_msg( handle, 0x220, req_diag_result_msg );
            if( wait_for_msg( handle, 0x239, 1000, data ) == 0x239 )
            {
                printf("\nDiagnostic results...\n");
                for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                printf("\n");
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                return -1;
            }
*/
        }
        else
        {
            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
            fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(512*1024)*100.0);
            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
            return -1;
        }
    }
    else
    {
        printf("err line: %d\n", __LINE__ );
        fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(512*1024)*100.0);
        fprintf( log_output, "err line: %d\n", __LINE__ );
        return -1;
    }
    
    return 0;
}

int program_trionic_tis( CANHANDLE handle, unsigned char *bin, const char *vin, const char *swdate, const char *tester )
{
    const char jump_msg1a[8]   = { 0x41, 0xA1, 0x08, 0x34, 0x00, 0x00, 0x00, 0x00 };    // 0x000000 length=0x070000
    const char jump_msg1b[8]   = { 0x00, 0xA1, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char jump_msg2a[8]   = { 0x41, 0xA1, 0x08, 0x34, 0x07, 0xFF, 0x00, 0x00 };    // 0x07FF00 length=0x000100
    const char jump_msg2b[8]   = { 0x00, 0xA1, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    const char end_data_msg[8] = { 0x40, 0xA1, 0x01, 0x37, 0x00, 0x00, 0x00, 0x00 };
    const char exit_diag_msg[8]= { 0x40, 0xA1, 0x02, 0x31, 0x54, 0x00, 0x00, 0x00 };
    const char req_diag_result_msg[8]= { 0x3F, 0x81, 0x01, 0x33, 0x02, 0x40, 0x00, 0x00 };  // 220h
    unsigned char ack[8]       = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };        // 266h
    unsigned char data[8];
    int i, k, bin_count;
    //HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    //CONSOLE_SCREEN_BUFFER_INFO csbi;
    

    bin_count = 0;
    //GetConsoleScreenBufferInfo(hout, &csbi);

    // Send "Request Download - tool to module" to Trionic
    send_msg( handle, 0x240, jump_msg1a );
    send_msg( handle, 0x240, jump_msg1b );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        
        if( data[3] == 0x74 )
        {
            // Send 0x00000...0x6FF90 (1911 message groups)
            while( (bin_count+240) < 0x70000 )
            {
                if ( bin_count % 0x147A )
                {
                    //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                    printf("%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                    //printf(".");
                }
                data[1] = 0xA1;
                for( i = 0x28; i >= 0; i-- )
                {
                    data[0] = i;
                    if( i == 0x28 )
                    {
                        data[0] |= 0x40;
                        data[2] = 0xF1; // length
                        data[3] = 0x36; // Data Transfer
                        data[4] = *(bin + bin_count++);
                        data[5] = *(bin + bin_count++);
                        data[6] = *(bin + bin_count++);
                        data[7] = *(bin + bin_count++);
                    }
                    else if ( i == 0 )
                    {
                        data[2] = *(bin + bin_count++);
                        data[3] = *(bin + bin_count++);
                        for( k = 4; k < 8; k++ ) data[k] = 0x00;
                    }
                    else
                    {
                        for( k = 2; k < 8; k++ )
                        {
                            data[k] = *(bin + bin_count++);
                        }
                    }
                    //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                    //printf("\n");
                    usleep(3000); //sleep(3);
                    send_msg( handle, 0x240, data );
                }
                // Read response
                if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                {
                    // Send acknowledgement
                    ack[3] = data[0] & 0xBF;
                    send_msg( handle, 0x266, ack );
                    if( data[3] != 0x76 )
                    {
                        // failed...
                        printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                        fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                        return -1;
                    }
                }
                else
                {
                    // failed...
                    printf("err line: %d\n", __LINE__ );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                    fprintf( log_output, "err line: %d\n", __LINE__ );
                    return -1;
                }
            }
            // Send 0x6FF91...0x70000
            data[1] = 0xA1;
            for( i = 0x12; i >= 0; i-- )
            {
                if ( bin_count % 0x147A )
                {
                    //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                    printf("%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                    //printf("o");
                }
                data[0] = i;
                if( i == 0x12 )
                {
                    data[0] |= 0x40;
                    data[2] = 0x71; // length
                    data[3] = 0x36; // Data Transfer
                    data[4] = *(bin + bin_count++);
                    data[5] = *(bin + bin_count++);
                    data[6] = *(bin + bin_count++);
                    data[7] = *(bin + bin_count++);
                }
                //else if( i == 0 )
                //{
                //    data[2] = *(bin + bin_count++);
                //    for( k = 3; k < 8; k++ ) data[k] = 0x00;
                //}
                else
                {
                    for( k = 2; k < 8; k++ )
                    {
                        data[k] = *(bin + bin_count++);
                    }
                }
                //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                //printf("\n");
                usleep(3000); //sleep(3);
                send_msg( handle, 0x240, data );
            }
            // Read response
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                if( data[3] != 0x76 )
                {
                    // failed...
                    printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                    fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                    return -1;
                }
            }
            else
            {
                // failed...
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }

            // Send "Request Download - tool to module" to Trionic
            // (i.e. jump to address 0x7FF00)
            send_msg( handle, 0x240, jump_msg2a );
            send_msg( handle, 0x240, jump_msg2b );
            
            bin_count = 0x70000;
        
            // Read response
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                
                if( data[3] = 0x74 )
                {
                    // Send 0x7FF00...7FFF0
                    data[1] = 0xA1;
                    for( i = 0x28; i >= 0; i-- )
                    {
                        if ( bin_count % 0x147A )
                        {
                            //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                            printf("%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                            //printf("#");
                        }
                        data[0] = i;
                        if( i == 0x28 )
                        {
                            data[0] |= 0x40;
                            data[2] = 0xF1; // length
                            data[3] = 0x36; // Data Transfer
                            data[4] = *(bin + bin_count++);
                            data[5] = *(bin + bin_count++);
                            data[6] = *(bin + bin_count++);
                            data[7] = *(bin + bin_count++);
                        }
                        else if ( i == 0 )
                        {
                            data[2] = *(bin + bin_count++);
                            data[3] = *(bin + bin_count++);
                            for( k = 4; k < 8; k++ ) data[k] = 0x00;
                        }
                        else
                        {
                            for( k = 2; k < 8; k++ )
                            {
                                data[k] = *(bin + bin_count++);
                            }
                        }
                        //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                        //printf("\n");
                        usleep(3000); //sleep(3);
                        send_msg( handle, 0x240, data );
                    }
                    // Read response
                    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                    {
                        // Send acknowledgement
                        ack[3] = data[0] & 0xBF;
                        send_msg( handle, 0x266, ack );
                        if( data[3] != 0x76 )
                        {
                            // failed...
                            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                            return -1;
                        }
                    }
                    else
                    {
                        // failed...
                        printf("err line: %d\n", __LINE__ );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                        fprintf( log_output, "err line: %d\n", __LINE__ );
                        return -1;
                    }
                    // Send 0x7FFF0...0x7FFFFF
                    data[1] = 0xA1;
                    for( i = 0x2; i >= 0; i-- )
                    {
                        if ( bin_count % 0x147A )
                        {
                            //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
                            printf("%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                            //printf("x");
                        }
                        data[0] = i;
                        if( i == 0x2 )
                        {
                            data[0] |= 0x40;
                            data[2] = 0x11; // length
                            data[3] = 0x36; // Data Transfer
                            data[4] = *(bin + bin_count++);
                            data[5] = *(bin + bin_count++);
                            data[6] = *(bin + bin_count++);
                            data[7] = *(bin + bin_count++);
                        }
                        else
                        {
                            for( k = 2; k < 8; k++ )
                            {
                                data[k] = *(bin + bin_count++);
                            }
                        }
                        //for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                        //printf("\n");
                        usleep(3000); //sleep(3);
                        send_msg( handle, 0x240, data );
                    }
                    // Read response
                    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
                    {
                        // Send acknowledgement
                        ack[3] = data[0] & 0xBF;
                        send_msg( handle, 0x266, ack );
                        if( data[3] != 0x76 )
                        {
                            // failed...
                            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                            return -1;
                        }
                    }
                    else
                    {
                        // failed...
                        printf("err line: %d\n", __LINE__ );
                        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                        fprintf( log_output, "err line: %d\n", __LINE__ );
                        return -1;
                    }

                }
                else
                {
                    printf("err line: %d\n", __LINE__ );
                    fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                    fprintf( log_output, "err line: %d\n", __LINE__ );
                    return -1;
                }
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }
            

        }
        else
        {
            printf("err line: %d\n", __LINE__ );
            fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
            fprintf( log_output, "err line: %d\n", __LINE__ );
            return -1;
        }
    }
    else
    {
        printf("err line: %d\n", __LINE__ );
        fprintf( log_output, "%5.1f %% done", (float)bin_count/(float)(0x70100)*100.0);
        fprintf( log_output, "err line: %d\n", __LINE__ );
        return -1;
    }

   // Send "Request Data Transfer Exit" to Trionic
    send_msg( handle, 0x240, end_data_msg );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        if( data[3] == 0x77 )
        {
            // Program VIN
            write_data_block( handle, 0x90, vin);
            // Program software date
            write_data_block( handle, 0x99, swdate);
            // Program tester info
            write_data_block( handle, 0x98, tester);

            // Send "Exit diagnostic routine"
            send_msg( handle, 0x240, exit_diag_msg );
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
                if( data[3] != 0x71 )
                {
                    printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
                    fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(0x70100)*100.0);
                    fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
                    return -1;
                }             
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(0x70100)*100.0);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }
/*
            // Sleep 5 seconds
            sleep(5000);
            
            // Send "Request diagnostic results"
            send_msg( handle, 0x220, req_diag_result_msg );
            if( wait_for_msg( handle, 0x239, 1000, data ) == 0x239 )
            {
                printf("\nDiagnostic results...\n");
                for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
                printf("\n");
            }
            else
            {
                printf("err line: %d\n", __LINE__ );
                return -1;
            }
*/
        }
        else
        {
            printf("err line: %d (0x%02X)\n", __LINE__, data[3] );
            fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(0x70100)*100.0);
            fprintf( log_output, "err line: %d (0x%02X)\n", __LINE__, data[3] );
            return -1;
        }
    }
    else
    {
        printf("err line: %d\n", __LINE__ );
        fprintf( log_output, "%5.1f %% done\n", (float)bin_count/(float)(0x70100)*100.0);
        fprintf( log_output, "err line: %d\n", __LINE__ );
        return -1;
    }
    
    return 0;
}

int write_data_block( CANHANDLE handle, unsigned char header_id, const unsigned char *block)
{
    unsigned char data[8], length, rows;
    int i, k;
    float row_temp;
    unsigned char write[8] = { 0x40, 0xA1, 0x00, 0x3B, 0x00, 0x00, 0x00, 0x00 };
    unsigned char ack[8]   = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    
   
    length = strlen(block);
    row_temp = floor( (float)( length + 3 ) / 6.0 );
    rows = (unsigned char)row_temp;
    
    // Send "Write data block" to Trionic
    write[2] = length + 2;
    write[4] = header_id;
    
    for( i = rows; i >= 0; i-- )
    {
        if( i == rows )
        {
            write[0] = i | 0x40;
            write[5] = *block++;
            write[6] = *block++;
            write[7] = *block++;
        }
        else
        {
            write[0] = i;
            for( k = 2; k < 8; k++ )
            {
                if( *block != 0 ) write[k] = *block++;
                else write[k] = 0x00;
            }
        }
        
        send_msg( handle, 0x240, write );
        //for( k = 0; k < 8; k++ ) printf("0x%02X ", write[k]);
        //printf("\n");
    }

    // Read response message

    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        if( data[3] == 0x7B && data[4] == header_id )
        {
            // ok
        }
        else
        {
            printf("err line: %d (0x%02X, 0x%02X)\n", __LINE__, data[3],data[4] );
            fprintf( log_output, "err line: %d (0x%02X, 0x%02X)\n", __LINE__, data[3],data[4] );
            return -1;
        }
    }
    else
    {
        printf("err line: %d\n", __LINE__ );
        fprintf( log_output, "err line: %d\n", __LINE__ );
        return -1;
    }
    
    return 0;
    
}

int read_trionic( CANHANDLE handle, int addr, int len, unsigned char *bin)
{
    unsigned char data[8], i, k;
    int address, length, rcv_len, dot, bytes_this_round, retries, ret;
    const char init_msg[8]     = { 0x20, 0x81, 0x00, 0x11, 0x02, 0x42, 0x00, 0x00 };
    const char end_data_msg[8] = { 0x40, 0xA1, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00 };
    char jump_msg1a[8]         = { 0x41, 0xA1, 0x08, 0x2C, 0xF0, 0x03, 0x00, 0xEF };    // 0x000000 length=0xEF
    char jump_msg1b[8]         = { 0x00, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char post_jump_msg[8]= { 0x40, 0xA1, 0x01, 0x3E, 0x00, 0x00, 0x00, 0x00 };
    const char data_msg[8]     = { 0x40, 0xA1, 0x02, 0x21, 0xF0, 0x00, 0x00, 0x00 };
    unsigned char ack[8]       = { 0x40, 0xA1, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
    //HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    //CONSOLE_SCREEN_BUFFER_INFO csbi;
    

    //GetConsoleScreenBufferInfo(hout, &csbi);
    rcv_len = 0;
    data[0] = 0x00;
    dot = 0;
    retries = 0;

    address = addr;

    while( rcv_len < len )
    {
        bytes_this_round = 0;
        if( (len - rcv_len) < 0xEF ) jump_msg1a[7] = len - rcv_len;
        else jump_msg1a[7] = 0xEF;
        
        jump_msg1b[2] = (address >> 16) & 0xFF;
        jump_msg1b[3] = (address >> 8) & 0xFF;
        jump_msg1b[4] = address & 0xFF;
    
        // Send read address and length to Trionic
        ret = send_msg( handle, 0x240, jump_msg1a );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'jump_msg1a' failed, err %d.\n", ret);
            fprintf( log_output, "Send 'jump_msg1a' failed, err %d.\n", ret);
            usleep(10000); //sleep(100);
            // Retry query to Trionic
            ret = send_msg( handle, 0x240, jump_msg1a );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'jump_msg1a' retry failed, err %d.\n", ret);
                fprintf( log_output, "Send 'jump_msg1a' retry failed, err %d.\n", ret);
            }
        }
        ret = send_msg( handle, 0x240, jump_msg1b );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'jump_msg1b' failed, err %d.\n", ret);
            fprintf( log_output, "Send 'jump_msg1b' failed, err %d.\n", ret);
            usleep(10000); //sleep(100);
            // Retry query to Trionic
            ret = send_msg( handle, 0x240, jump_msg1b );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'jump_msg1b' retry failed, err %d.\n", ret);
                fprintf( log_output, "Send 'jump_msg1b' retry failed, err %d.\n", ret);
            }
        }
    
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
			//printf("wait_for_msg( handle, 0x258, 1000, data ) == 0x258");
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
            
            if( data[3] != 0x6C || data[4] != 0xF0 )
            {
                printf("err line: %d\n", __LINE__ );
                fprintf( log_output, "%5.1f %% done (retries = %d)\n", (float)rcv_len/(float)len*100.0, retries);
                fprintf( log_output, "err line: %d\n", __LINE__ );
                return -1;
            }
        }
        else
        {
            printf("err line: %d\n", __LINE__ );
            fprintf( log_output, "%5.1f %% done (retries = %d)\n", (float)rcv_len/(float)len*100.0, retries);
            fprintf( log_output, "err line: %d\n", __LINE__ );
            return -1;
        }
/*    
        // Send post jump message to Trionic
        send_msg( handle, 0x240, post_jump_msg );
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
            
            if( data[3] != 0x7E )
            {
                printf("err line: %d\n", __LINE__ );
                return -1;
            }
        }
        else
        {
            printf("err line: %d\n", __LINE__ );
            return -1;
        }
*/        
		//printf("send data transfer to trionic\n");
        // Send "Data Transfer" to Trionic
        send_msg( handle, 0x240, data_msg );
        
        // Read response messages
        data[0] = 0x00;
        while( data[0] != 0x80 && data[0] != 0xC0 )
        {
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                //for( i = 0; i < 8; i++ ) printf("0x%02X ", data[i]);
                //printf("\n");
                if( data[0] & 0x40 )
                {
                    length = data[2] - 2;   // subtract two non-payload bytes
                    if( --length > 0 && rcv_len < len )
                    {
                        *bin++ = data[5];
                        rcv_len++;
                        bytes_this_round++;
                    }
                    if( --length > 0 && rcv_len < len )
                    {
                        *bin++ = data[6];
                        rcv_len++;
                        bytes_this_round++;
                    }
                    if( --length > 0 && rcv_len < len )
                    {
                        *bin++ = data[7];
                        rcv_len++;
                        bytes_this_round++;
                    }
                }
                else
                {
                    for( i = 0; i < 6; i++ )
                    {
                        if( rcv_len < len )
                        {
                            *bin++ = data[2+i];
                            rcv_len++;
                            bytes_this_round++;
                            length--;
                            if( length == 0 ) i = 6;
                        }
                    }
                }
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                ret = send_msg( handle, 0x266, ack );
                if( ret != ERROR_CANUSB_OK )
                {
                    printf("Send 'ack' failed, err %d.\n", ret);
                    fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
                    usleep(10000); //sleep(100);
                    // Retry acknowledgement
                    ret = send_msg( handle, 0x266, ack );
                    if( ret != ERROR_CANUSB_OK )
                    {
                        printf("Send 'ack' retry failed, err %d.\n", ret);
                        fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
                    }
                }
            }
            else
            {
                // Timeout
                retries++;
                if( retries < 10 )
                {
                    rcv_len -= bytes_this_round;
                    bin -= bytes_this_round;
                    length += bytes_this_round;
                    // retry jump addr + data transfer command
                    break;
                }
                else
                {
                    printf("\nerr line: %d\n", __LINE__ );
                    fprintf( log_output, "%5.1f %% done (retries = %d)\n", (float)rcv_len/(float)len*100.0, retries);
                    fprintf( log_output, "\nerr line: %d\n", __LINE__ );
                    return -1;
                }
            }
        }
        
        address = addr + rcv_len;
		//printf("address 0x%X\n",address);
		//printf("bytesthisround=%d\n",bytes_this_round);
        //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
        if( retries == 0 )
        {
            printf("%5.1f %% done\n", (float)rcv_len/(float)len*100.0);
        }
        else
        {
            printf("%5.1f %% done (retries = %d)\n", (float)rcv_len/(float)len*100.0, retries);
        }
    }
    
   // Send "Request Data Transfer Exit" to Trionic
    send_msg( handle, 0x240, end_data_msg );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        ret = send_msg( handle, 0x266, ack );
        if( ret != ERROR_CANUSB_OK )
        {
            printf("Send 'ack' failed, err %d.\n", ret);
            fprintf( log_output, "Send 'ack' failed, err %d.\n", ret);
            usleep(10000); //sleep(100);
            // Retry query to Trionic
            ret = send_msg( handle, 0x266, ack );
            if( ret != ERROR_CANUSB_OK )
            {
                printf("Send 'ack' retry failed, err %d.\n", ret);
                fprintf( log_output, "Send 'ack' retry failed, err %d.\n", ret);
            }
        }
        if( data[3] != 0xC2 )
        {
            fprintf( log_output, "%5.1f %% done (retries = %d)", (float)rcv_len/(float)len*100.0, retries);
            for( k = 0; k < 8; k++ )
            {
                printf("0x%02X ", data[k]);
                fprintf( log_output, "0x%02X ", data[k]);
            }
            printf("\nerr line: %d\n", __LINE__ );
            fprintf( log_output, "\nerr line: %d\n", __LINE__ );
            return -1;
        }
    }

    return rcv_len;
}

// TODO: Verify needs to take into account the way the header part is written
// i.e. the written binary will not be exactly the same as the one read
int verify_trionic( CANHANDLE handle, int addr, int len, const unsigned char *written)
{
    unsigned char data[8], i, k;
    int address, length, rcv_len, dot;
    const char init_msg[8]     = { 0x20, 0x81, 0x00, 0x11, 0x02, 0x42, 0x00, 0x00 };
    const char end_data_msg[8] = { 0x40, 0xA1, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00 };
    char jump_msg1a[8]         = { 0x41, 0xA1, 0x08, 0x2C, 0xF0, 0x03, 0x00, 0xEF };    // 0x000000 length=0xEF
    char jump_msg1b[8]         = { 0x00, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char post_jump_msg[8]= { 0x40, 0xA1, 0x01, 0x3E, 0x00, 0x00, 0x00, 0x00 };
    const char data_msg[8]     = { 0x40, 0xA1, 0x02, 0x21, 0xF0, 0x00, 0x00, 0x00 };
    unsigned char ack[8]       = { 0x40, 0xA1, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
   // HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
   // CONSOLE_SCREEN_BUFFER_INFO csbi;
    

    //GetConsoleScreenBufferInfo(hout, &csbi);
    rcv_len = 0;
    data[0] = 0x00;
    dot = 0;

    address = addr;

    while( rcv_len < len )
    {
        if( (len - rcv_len) < 0xEF ) jump_msg1a[7] = len - rcv_len;
        else jump_msg1a[7] = 0xEF;
        
        jump_msg1b[2] = (address >> 16) & 0xFF;
        jump_msg1b[3] = (address >> 8) & 0xFF;
        jump_msg1b[4] = address & 0xFF;
    
        // Send read address and length to Trionic
        send_msg( handle, 0x240, jump_msg1a );
        send_msg( handle, 0x240, jump_msg1b );
    
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
            
            if( data[3] != 0x6C || data[4] != 0xF0 )
            {
                printf("err line: %d\n", __LINE__ );
                return -1;
            }
        }
        else
        {
            printf("err line: %d\n", __LINE__ );
            return -1;
        }

        // Send "Data Transfer" to Trionic
        send_msg( handle, 0x240, data_msg );
        
        // Read response messages
        data[0] = 0x00;
        while( data[0] != 0x80 && data[0] != 0xC0 )
        {
            if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
            {
                if( data[0] & 0x40 )
                {
                    length = data[2] - 2;   // subtract two non-payload bytes
                    if( --length > 0 && rcv_len < len )
                    {
                        if( *written++ != data[5] ) return -1;
                        rcv_len++;
                    }
                    if( --length > 0 && rcv_len < len )
                    {
                        if( *written++ != data[6] ) return -1;
                        rcv_len++;
                    }
                    if( --length > 0 && rcv_len < len )
                    {
                        if( *written++ != data[7] ) return -1;
                        rcv_len++;
                    }
                }
                else
                {
                    for( i = 0; i < 6; i++ )
                    {
                        if( rcv_len < len )
                        {
                            if( *written++ != data[2+i] ) return -1;
                            rcv_len++;
                            length--;
                            if( length == 0 ) i = 6;
                        }
                    }
                }
                // Send acknowledgement
                ack[3] = data[0] & 0xBF;
                send_msg( handle, 0x266, ack );
            }
            else
            {
                // Timeout
                printf("err line: %d\n", __LINE__ );
                return -1;
            }
        }
        
        address = addr + rcv_len;
        //SetConsoleCursorPosition( hout, csbi.dwCursorPosition );
        printf("%5.1f %% done", (float)rcv_len/(float)len*100.0);
    }
    
   // Send "Request Data Transfer Exit" to Trionic
    send_msg( handle, 0x240, end_data_msg );

    // Read response
    if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
    {
        // Send acknowledgement
        ack[3] = data[0] & 0xBF;
        send_msg( handle, 0x266, ack );
        if( data[3] != 0xC2 )
        {
            for( k = 0; k < 8; k++ ) printf("0x%02X ", data[k]);
            printf("\n");
            printf("err line: %d\n", __LINE__ );
            return -1;
        }
    }

    return rcv_len;
}


void ask_header2( CANHANDLE handle, unsigned char header_id, unsigned char *answer)
{
    unsigned char data[8], length, i;
    unsigned char query[8] = { 0x40, 0xA1, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00 };
    unsigned char ack[8]   = { 0x40, 0xA1, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    
   
    query[4] = header_id;
    data[0] = 0x00;
    
    // Send query to Trionic
    send_msg( handle, 0x240, query );

    // Read response messages
    while( data[0] != 0x80 && data[0] != 0xC0 )
    {
        if( wait_for_msg( handle, 0x258, 1000, data ) == 0x258 )
        {
            for( i = 0; i < 8; i++ ) printf("0x%02X ", data[i]);
            printf("\n");
            if( data[0] & 0x40 )
            {
                length = data[2] - 2;   // subtract two non-payload bytes
                if( --length > 0 ) *answer++ = data[5];
                if( --length > 0 ) *answer++ = data[6];
                if( --length > 0 ) *answer++ = data[7];
            }
            else
            {
                for( i = 0; i < 6; i++ )
                {
                    *answer++ = data[2+i];
                    length--;
                    if( length == 0 ) i = 6;
                }
            }
            // Send acknowledgement
            ack[3] = data[0] & 0xBF;
            send_msg( handle, 0x266, ack );
        }
        else
        {
            // Timeout
            printf("Timeout waiting for 0x258.\n");
            return;
        }
    }

    // Set end of string
    *answer = 0;
}

int send_msg( CANHANDLE handle, int id, const unsigned char *data )
{
    CANMsg msg;
    
    msg.id = id;
    msg.len = 8;
    msg.flags = 0;
    msg.data[0] = data[0];
    msg.data[1] = data[1];
    msg.data[2] = data[2];
    msg.data[3] = data[3];
    msg.data[4] = data[4];
    msg.data[5] = data[5];
    msg.data[6] = data[6];
    msg.data[7] = data[7];
    
    return sendFrame( handle, &msg );//canusb_Write( handle, &msg );
}

int wait_for_msg( FT_HANDLE handle, int id, int timeout, unsigned char *data )
{
    CANMsg msg;
    int timeout_temp;
    char not_received, got_it;
    long dwStart;
    
    dwStart = gettickscount();
    timeout_temp = timeout;
    msg.id = 0x0;
    not_received = 1;
    while( not_received == 1 )
    {
		if( readFrame ( handle, &msg ) )
		{
			if( msg.id == id || id == 0 )
			{
				not_received = 0;
				*(data+0) = msg.data[0];
				*(data+1) = msg.data[1];
				*(data+2) = msg.data[2];
				*(data+3) = msg.data[3];
				*(data+4) = msg.data[4];
				*(data+5) = msg.data[5];
				*(data+6) = msg.data[6];
				*(data+7) = msg.data[7];
				break;
			}
		}
		//else
		//{
			//printf(".");
		//}

        //if( (gettickscount() - dwStart) > timeout )
		//{
		//	not_received = 0;	
		//}
		//if( --timeout <= 0 )
		//	not_received = 0;
        else 
			usleep(1000);
    }
	//printf("msg.id=%X timeout time: %d\n", msg.id, gettickscount() - dwStart);
    return msg.id;
}

int get_header_field_string(const unsigned char *bin, unsigned char id, unsigned char *answer)
{
     unsigned char byte, length_field, id_field, found_id;
     unsigned char data[255];
     unsigned int addr;
     int i;

     //addr = 0x7FFFF;
     found_id = 0;
     addr = binary_length - 1;
    /* Reads backwards from the end of the binary */
    //while( addr > 0x7FE00 )
    //printf("trying to find 0x%02X\n", id);
    while( addr > (binary_length - 0x1FF) )
    {
        /* The first byte is the length of the data */
        length_field = *(bin+addr);
        //printf("%3d, ", length_field);
        if( length_field == 0x00 || length_field == 0xFF ) break;
        addr--;
        
        /* Second byte is an ID field */
        id_field = *(bin+addr);
        addr--;

        //printf("0x%02X\n", id_field);

        /* The actual data */
        for( i = 0; i < length_field; i++ )
        {
            if( id_field == id )
            {
                answer[i] = *(bin+addr);
            }
            addr--;
        }
        if( id_field == id )
        {
            answer[length_field] = 0;
            found_id = 1;
            // when this return is commented out, the function will
            // find the last field if there are several (mainly this
            // is for searching for the last VIN field)
            //return 1;
        }
    }
    
    return found_id;
}

int strip_header_field(unsigned char *bin)
{
     unsigned char byte, length_field, id_field;
     unsigned char data[255];
     unsigned int addr;
     int i;

     addr = 0x7FFFF;
     
    /* Reads backwards from the end of the binary */
    while( addr > 0x7FD00 )
    {
        /* The first byte is the length of the data */
        length_field = *(bin+addr);
        if( length_field == 0x00 || length_field == 0xFF ) break;
        addr--;
        
        /* Second byte is an ID field */
        id_field = *(bin+addr);
        addr--;

        //if( id_field == 0xFE )
        if( id_field == 0x92 )
        {
            // remove rest of the header leaving the 0xFE id field the last one
            addr -= length_field;
            while( addr > 0x7FD00 )
            {
                *(bin+addr) = 0xFF;
                addr--;
            }
            return 1;
        }
        /* The actual data */
        addr -= length_field;
    }
    
    return 0;
}

long gettickscount()
{
	struct timeval now;
	gettimeofday(&now, NULL);
	long ticks = now.tv_sec * 1000l;
	ticks += now.tv_usec / 1000l;
	return ticks;
}
