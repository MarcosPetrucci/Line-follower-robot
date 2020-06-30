#include <QTRSensors.h>

// ------- DECLARACAO DE VARIAVEIS ------- //

// ---- Sinal PWM do motor ---- //
const int motor1 = 1;
const int motor2 = 2;

// ---- Sensores Laterais --- //
const int dir = 8;
const int esq = 9;
const int CONTRASTE = 800; //Dado que serão um único sensor analógico, a leitura varia de 0 - 1023. Devemos definir um valor alto para o contraste ser preciso

// ----- Variáveis relativas ao PID ----- //

// ---- Sensores frontais --- //
const int front1 = 3;
const int front2 = 4;
const int front3 = 5;
const int front4 = 6;
const int front5 = 7;
const uint8_t numSensores = 5;
uint16_t  valorSensores[numSensores];

//Criar o objeto barra de sensores
QTRSensors qtr;

//Função que calibra a barra de sensores
void calibraBarraSensores() 
{
  delay(300);
  for (int i = 0; i < 300; i++)  // A calibragem demora em torno de 5 a 10s (aparentemente)
  { 
    qtr.calibrate();      //Calibra os sensores
  }
}

const int qt_dir = 5; //Define a quantidade de faixas da direita
const int qt_esq  =5; //Define a quantidade de faixas da esquerda

int indc_dir = 0; //Armazena o indice do vetor de faixas da direita
int indc_esq = 0; //Armazena o indice do vetor de faixas da esquerda

int faixa_dir[qt_dir]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro - Indicará se a proxima leitura corresponde a inicio, cruzamento ou fim
int faixa_esq[qt_esq]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro - Indicará se a proxima leitura corresponde a curva ou reta
/*
 * OBS: A posição 0 destes vetores vão ser consideradas. O que significa que, a cada nova leitura, eu vou considerar um novo estado válido; e só depois de processar tudo que incrementa o índice 
 */
//!DEFINIMOS:
//Para o vetor da esquerda
const int RETA = 0;
const int CURVA = 1;

//Para o vetor da direita
const int SEGUE = 1;
const int FIM = 0;
//const int CRUZAMENTO = 1 A princípio não preciso definir o cruzamento. Basta mandar o robô seguir.

// Kp Ki Kd ideais para reta e curva
double PID_reta[3] = {0, 0, 0};
double PID_curva[3] = {0, 0, 0};

//Variaveis de uso na loop
int direita; //Armazena a leitura atual da direita
int esquerda; //Armazena a leitura atual da esquerda
int parar = 0; //Aparentemente não há um jeito de desligar o arduino via código, por isso se esta variável for 1 o robô não fará nada
double erro;


// ------------------- PID ------------------- //
//Creditos: Ivan Seidel
//Inspirado de: https://gist.github.com/ivanseidel/b1693a3be7bb38ff3b63

//Classe PID com todos os métodos e atributos necessários
class PID{
  
  private:
    double erro;
    uint16_t posicao;
    uint16_t posiAnt;
    double kP, kI, kD;      
    double P, I, D;
    double pid;

    //Dado a propriedade do array de sensores, 2000 é o ponto de equilíbrio perfeito
    double setPoint = 2000;
    long tempoAnt;  

  public:

  void setKPID(int estado)
  {
    //Considerando uma leitura de rotina, ou seja, a leitura de faixas da esquerda/direita está prevista em outro ponto do programa
    if(!estado) //Reta
    {
      kP = PID_reta[0];
      kI = PID_reta[1];
      kD = PID_reta[2];
    }
    else //Curva
    {
      kP = PID_curva[0];
      kI = PID_curva[1];
      kD = PID_curva[2];
    }
  }
  
  double getErro()
  {
    //(for a white line, use readLineWhite() instead)    
    uint16_t posicao = qtr.readLineBlack(valorSensores); //Le os sensores e retorna um valor de 0 até 4000
    
    // Implementação PID
    erro = setPoint - posicao;
    float deltaTime = (millis() - tempoAnt) / 1000.0;
    tempoAnt = millis();
    
    //P
    P = erro * kP;
    
    //I
    I = I + (erro * kI) * deltaTime;
    
    //D
    D = (posiAnt - posicao) * kD / deltaTime;
    posiAnt = posicao;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

//Cria objeto PID
PID leitor;

void setup() 
{
  // --- Setando pinos ---//
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(front1, INPUT);
  pinMode(front1, INPUT);
  pinMode(front2, INPUT);
  pinMode(front3, INPUT);
  pinMode(front4, INPUT);
  pinMode(front5, INPUT);
  pinMode(esq, INPUT);
  pinMode(dir, INPUT); 

  //Configurar barra de sensores
  //qtr.setTypeAnalog();
  qtr.setTypeRC(); // Não sei exatamente qual dos dois modos de operação
  qtr.setSensorPins((const uint8_t[]) {front1, front2, front3, front4, front5}, numSensores);  //5 Sensores - Valor flutua de 0 até 4000 - SetPoint será 2000
  calibraBarraSensores();
}

void loop() 
{
  if(parar)
    return;
    
  /* Evitar alocação no loop - poupa tempo */
  //Inicia a volta
  direita = analogRead(dir); //Retornam valores analógicos na range de 0 - 1023
  esquerda = analogRead(esq);
  
  //Problema importante: E se o programa entrar várias vezes nesses IFs ? Como filtrar? Colocar um intervalo de tempo mínimo de tolerância?
    //  Resposta: Só com testes saberemos....
  if(esquerda >= CONTRASTE)
  {
    if(faixa_esq[indc_esq] == RETA) //Estamos em uma reta
    {
      leitor.setKPID(RETA);
    }
    else //Estamos em uma curva                        
    {
      leitor.setKPID(CURVA);
    }

    erro = leitor.getErro();
    //COMUNICAR MOTORES
    
    indc_esq++;
  }

  //O programa DEVE entrar nos dois IFs, pois o vetor da direita deve estar devidamente atualizado para pararmos na hora certa
  if(direita >= CONTRASTE)
  {
    //Não muda PID, esta responsabilidade cabe ao sensor da esquerda 
    if(faixa_esq[indc_esq] == FIM) //Chegamos ao FIM do trajeto, devemos parar!!
    {
      parar = 1;

      //!PARAR O MOTOR ANTES DE SAIR DESSA ITERAÇÃO 
    }


   
    
    indc_dir++;
  }  
}
