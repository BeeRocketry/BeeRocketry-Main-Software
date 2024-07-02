/*
--------------------------------------------------------------------
 Debug Printer Makrosu
    DEBUG_MODE makrosu tanimlandığında debug mesajlarının
    seri porta yazdırılmasını sağlayan makro tanımıdır.

        DEBUG_PRINTER --> Yazdırılacak olan seri port aygıtını seçer
--------------------------------------------------------------------
*/

#define DEBUG_PRINTER Serial1

#ifdef DEBUG_MODE
    #define DEBUG_PRINT(...) {DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) {DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif