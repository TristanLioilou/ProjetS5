#define _DEFAULT_SOURCE
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

const char *portTTY = "/dev/ttyS1";  // Port série (modifier selon ta configuration)
const char *can_interface = "can0";  // Interface CAN (modifier selon ta configuration)

void setup_uart(int *fd) {
    struct termios SerialPortSettings;
    
    *fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
    if (*fd == -1) {
        perror("Erreur d'ouverture du port série");
        return;
    }

    tcgetattr(*fd, &SerialPortSettings);

    cfsetispeed(&SerialPortSettings, B115200);  // Vitesse de transmission en bauds
    cfsetospeed(&SerialPortSettings, B115200);

    // Mode 8N1 (8 bits, pas de parité, 1 bit de stop)
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_cflag &= ~CRTSCTS; // Pas de contrôle de flux matériel
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    // Désactivation du contrôle de flux logiciel (XON/XOFF)
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode non canonique
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Pas de traitement de sortie
    SerialPortSettings.c_oflag &= ~OPOST;

    if (tcsetattr(*fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur de configuration du port série");
        close(*fd);
        return;
    }
}

int setup_can_socket() {
    int fdSocketCAN;
    struct sockaddr_can addr;
    struct ifreq ifr;

    fdSocketCAN = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fdSocketCAN < 0) {
        perror("Erreur de création du socket CAN");
        return -1;
    }

    // Récupérer l'index de l'interface CAN
    strncpy(ifr.ifr_name, can_interface, sizeof(ifr.ifr_name) - 1);
    if (ioctl(fdSocketCAN, SIOCGIFINDEX, &ifr) < 0) {
        perror("Erreur ioctl");
        close(fdSocketCAN);
        return -1;
    }

    // Configuration de l'adresse CAN
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fdSocketCAN, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Erreur de liaison du socket CAN");
        close(fdSocketCAN);
        return -1;
    }

    return fdSocketCAN;
}

void send_can_message(int can_socket, uint32_t id, const char *data, int length) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = length;
    memcpy(frame.data, data, length);

    if (write(can_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Erreur d'envoi du message CAN");
    } else {
        printf("Message CAN ID 0x%x envoyé\n", id);
    }
}

void read_and_forward_can_to_uart(int can_socket, int uart_fd) {
    struct can_frame frame;
    int nbytes;

    while (1) {
        nbytes = read(can_socket, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Erreur de lecture du message CAN");
            continue;
        }

        // Vérifier si l'ID CAN est celui attendu (ex. 0x002, 0x456, 0x789)
        if (frame.can_id == 0x002 || frame.can_id == 0x456 || frame.can_id == 0x789) {
            write(uart_fd, frame.data, frame.can_dlc);
            printf("Message CAN ID 0x%x envoyé sur UART\n", frame.can_id);
        }
    }
}

void read_and_forward_uart_to_can(int uart_fd, int can_socket) {
    char buffer[32];
    int bytes_read;

    while (1) {
        bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            // Envoie des données sur le bus CAN avec l'ID 0x123
            send_can_message(can_socket, 0x123, buffer, bytes_read);
            printf("Message UART envoyé sur CAN\n");
        }
    }
}

int main(void) {
    int uart_fd, can_socket;

    // Initialisation du port série
    setup_uart(&uart_fd);
    if (uart_fd < 0) {
        return -1;
    }

    // Initialisation du socket CAN
    can_socket = setup_can_socket();
    if (can_socket < 0) {
        close(uart_fd);
        return -1;
    }

    printf("En attente de données sur CAN et UART...\n");

    // Lancer les deux tâches en parallèle
    pid_t pid = fork();
    if (pid == 0) {
        // Processus fils : lecture UART et envoi sur CAN
        read_and_forward_uart_to_can(uart_fd, can_socket);
    } else if (pid > 0) {
        // Processus parent : lecture CAN et envoi sur UART
        read_and_forward_can_to_uart(can_socket, uart_fd);
    } else {
        perror("Erreur de fork");
        close(uart_fd);
        close(can_socket);
        return -1;
    }

    // Fermer les descripteurs
    close(uart_fd);
    close(can_socket);

    return 0;
}
