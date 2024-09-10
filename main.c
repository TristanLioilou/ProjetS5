#define _GNU_SOURCE

#include <string.h>
#include <stdio.h>
#include <fcntl.h>   // Définitions du contrôle de fichiers
#include <termios.h> // Définitions POSIX de contrôle de terminal
#include <unistd.h>  // Définitions standard UNIX
#include <errno.h>   // Définitions des erreurs

// Port série à utiliser
const char *portTTY = "/dev/ttyS1";// Modifier le port en fonction de ta configuration

void main(void) {
    int fd; // Descripteur de fichier

    printf("\nLecture et écriture sur le port série\n");

    // Ouvrir le port série
    fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        printf("\nErreur d'ouverture de %s\n", portTTY);
        return;
    } else {
        printf("\nOuverture de %s réussie\n", portTTY);
    }

    // Configuration des attributs du port série
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings); // Récupérer les attributs actuels du port

    // Configurer la vitesse du port série
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // Mode 8N1 (8 bits, pas de parité, 1 bit de stop)
    SerialPortSettings.c_cflag &= ~PARENB;   // Pas de parité
    SerialPortSettings.c_cflag &= ~CSTOPB;   // 1 bit de stop
    SerialPortSettings.c_cflag &= ~CSIZE;    // Effacer le masque de taille de données
    SerialPortSettings.c_cflag |= CS8;       // 8 bits de données
    SerialPortSettings.c_cflag &= ~CRTSCTS;  // Pas de contrôle de flux matériel
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer la réception, ignorer les lignes de contrôle du modem

    // Désactiver le contrôle de flux logiciel (XON/XOFF)
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode non canonique (pas de traitement de ligne, pas d'écho)
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Pas de traitement de sortie
    SerialPortSettings.c_oflag &= ~OPOST;

    // Appliquer la configuration
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        printf("\nErreur de configuration des attributs du port série\n");
        close(fd);
        return;
    }

    // Lecture des données du port série
    char read_buffer[32]; // Buffer pour les données reçues
    int bytes_read = 0;

    printf("En attente de données...\n");

    while (1) {
        bytes_read = read(fd, read_buffer, sizeof(read_buffer)); // Lire les données
        if (bytes_read > 0) {
            printf("Bytes reçus : %d --> ", bytes_read);
            for (int i = 0; i < bytes_read; i++) {
                printf("%c", read_buffer[i]);
            }
            printf("\n");

            // Renvoi des mêmes données à l'envoyeur
            int bytes_written = write(fd, read_buffer, bytes_read);
            if (bytes_written != bytes_read) {
                printf("Erreur lors de l'écriture sur le port série\n");
            } else {
                printf("Données envoyées : %d octets\n", bytes_written);
            }
        }
    }

    // Fermer le port série
    close(fd);
    printf("\nFermeture du port série\n");
}
