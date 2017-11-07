#include <per_proto.h>
#include <pragmas.h>

int main()
{
    OFF_ALL_ANALOG_INPUTS
    uart_module_t a;
    a = UART_init(1,UART_460800, INT_PRIO_HIGHEST);
    UART_write_string(a,"Uart ok!\n");
    while(1)
    {
        if (UART_bytes_available(a))
        {
            char b=0;
            b = UART_get_byte(a);
            UART_write_string(a,"recieve: %c\n",b);
        }
        
//        delay_ms(100);
    }
    return 0;
}