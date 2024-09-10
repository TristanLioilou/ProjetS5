#define _GNU_SOURCE

#include <string.h>
#include <stdio.h>
#include <fcntl.h>    // Définitions du contrôle de fichiers
#include <termios.h>  // Définitions POSIX de contrôle de terminal
#include <unistd.h>   // Définitions standard UNIX
#include <errno.h>    // Définitions des erreurs
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

// Port série à utiliser
const char *portTTY = "/dev/ttyS1"; // Modifier le port en fonction de ta configuration
const char *can_interface = "can0"; // Interface CAN à utiliser

void setup_uart(int *fd) {
    struct termios SerialPortSettings;
    // Configuration du port série
    *fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
    if (*fd == -1) {
        printf("\nErreur d'ouverture de %s\n", portTTY);
        return;
    } else {
        printf("\nOuverture de %s réussie\n", portTTY);
    }

    tcgetattr(*fd, &SerialPortSettings); // Récupérer les attributs actuels du port

    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // Mode 8N1 (8 bits, pas de parité, 1 bit de stop)
    SerialPortSettings.c_cflag &= ~PARENB;   // Pas de parité
    SerialPortSettings.c_cflag &= ~CSTOPB;   // 1 bit de stop
    SerialPortSettings.c_cflag &= ~CSIZE;    // Effacer le masque de taille de données
    SerialPortSettings.c_cflag |= CS8;       // 8 bits de données
    SerialPortSettings.c_cflag &= ~CRTSCTS;  // Pas de contrôle de flux matériel
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer la réception

    // Désactiver le contrôle de flux logiciel (XON/XOFF)
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode non canonique
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Pas de traitement de sortie
    SerialPortSettings.c_oflag &= ~OPOST;

    if (tcsetattr(*fd, TCSANOW, &SerialPortSettings) != 0) {
        printf("\nErreur de configuration des attributs du port série\n");
        close(*fd);
        return;
    }
}

int setup_can_socket() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Créer le socket CAN
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Erreur de création du socket CAN");
        return -1;
    }

    // Ouvrir l'interface CAN
    strncpy(ifr.ifr_name, can_interface, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("Erreur dans l'ioctl");
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Erreur de liaison du socket CAN");
        return -1;
    }

    return s;
}

void send_can_message(int can_socket, uint32_t id, const char *data, int length) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = length;
    memcpy(frame.data, data, length);

    if (write(can_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Erreur d'envoi du message CAN");
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

        // Si l'ID CAN est 456 ou 789, renvoyer les données sur UART
        if (frame.can_id == 0x456 || frame.can_id == 0x789 ||frame.can_id == 0x123) {
            write(uart_fd, frame.data, frame.can_dlc);
            printf("Message CAN reçu de l'ID 0x%x et envoyé par UART\n", frame.can_id);
        }
    }
}

void read_and_forward_uart_to_can(int uart_fd, int can_socket) {
    char buffer[32];
    int bytes_read;

    while (1) {
        bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            // Envoyer les données sur le bus CAN avec l'adresse 123
            send_can_message(can_socket, 0x123, buffer, bytes_read);
            printf("Message UART reçu et envoyé sur CAN\n");
        }
    }
}

int main(void) {
    int uart_fd, can_socket;

    printf("\nLecture et écriture sur le port série et CAN\n");

    // Initialisation du port série
    setup_uart(&uart_fd);

    // Initialisation du socket CAN
    can_socket = setup_can_socket();
    if (can_socket == -1) {
        close(uart_fd);
        return 1;
    }

    printf("En attente de données sur CAN et UART...\n");

    // Lire du UART et envoyer sur CAN
    read_and_forward_uart_to_can(uart_fd, can_socket);

    // Lire du CAN et envoyer sur UART
    read_and_forward_can_to_uart(can_socket, uart_fd);

    // Fermer le port série et le socket CAN
    close(uart_fd);
    close(can_socket);

    printf("\nFermeture du port série et du socket CAN\n");

    return 0;
}
