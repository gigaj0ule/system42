#ifndef COMMANDS_H
    #define COMMANDS_H

    // TODO: resolve assert
    #define communication_assert(expr)

    #ifdef __cplusplus

        #include "protocol.hpp"

        auto make_protocol_definitions(void);

        extern "C" {
    #endif

            void pntp_begin(const char * default_hostname);
            int pntp_listen(void);
            int pntp_listen_eth0(void);
            int pntp_listen_tty0(void);

        #ifdef __cplusplus
        }
        #endif

#endif /* COMMANDS_H */
