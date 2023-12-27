#include "main.h"

int main()
{
     /*----------------------------- Ask Menu implementation----------------------------------------*/
    printf("\n\n +==========================================+");
    printf("\n |           STM32F4 BootLoader v1           |");
    printf("\n +==========================================+\n");


    //Not Implemented yet
    Serial_Port_Configuration();

    while(1)
    {
#if 1
        printf("\n\n +==========================================+");
        printf("\n |                   Menu                   |");
        printf("\n +==========================================+\n");
#endif
        printf("\n\n   Which BL command do you want to send ??\n");
        printf("\n   GET_VER                     --> 1");
        printf("\n   GET_HLP                     --> 2");
        printf("\n   GET_CID                     --> 3");
        printf("\n   GET_RDP_STATUS              --> 4");
        printf("\n   GO_TO_ADDR                  --> 5");
        printf("\n   FLASH_MASS_ERASE            --> 6");
        printf("\n   FLASH_ERASE                 --> 7");
        printf("\n   MEM_WRITE                   --> 8");
        printf("\n   EN_R_W_PROTECT              --> 9");
        printf("\n   MEM_READ                    --> 10");
        printf("\n   READ_SECTOR_P_STATUS        --> 11");
        printf("\n   OTP_READ                    --> 12");
        printf("\n   DIS_R_W_PROTECT             --> 13");
        printf("\n   MY_NEW_COMMAND              --> 14");
        printf("\n   MENU_EXIT                   --> 0");

        printf("\n\n   Type the command code here :");

        uint32_t command_code;
        scanf(" %d",&command_code);

        decode_menu_command_code(command_code);

#if 0
        printf("\n\n   Do you want to continue(y/n) ?:");
        uint8_t proceed = 0;
        scanf(" %c",&proceed);
        proceed -= 'y';
        if ( proceed)
        {
            printf("\n  ****** Thank you ! Exiting ******\n");
            break;
        }
#endif
        printf("\n\n   Press any key to continue  :");
        uint8_t ch = getch();
        purge_serial_port();
   }


}
