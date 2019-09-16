#pragma once
#ifndef CONSTANTES_H_INCLUDED
#define CONSTANTES_H_INCLUDED

#define PI 3.141593

enum team { NONE = 0, PURPLE, YELLOW };

//constantes de la fenetre
#define WINDOW_WIDTH 1453
#define WINDOW_HEIGHT 980
#define FPS 60

//constantes du robot
const int robotLength = 110;
const int robotWidth = 100;
#define ECART_ROUE 110
#define MAX_ACCELERATION 1

//constantes de la table
#define TABLE_LENGTH 3010
#define TABLE_HEIGTH 2150

#define NB_PALETS 0

//genetique
#define NB_SOL 5
#define NB_TOURS_1 5
#define NB_TOURS_SIMULES 10
#define TEMPS_GEN 150

const int temps_gen = 150;
const int nbToursSimules = 10;

#define TARGET_LIST_SIZE 100

#endif // CONSTANTES_H_INCLUDED
