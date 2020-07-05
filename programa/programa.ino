#include <QTRSensors.h>

// ------- DECLARACAO DE VARIAVEIS ------- //

// ---- Sinal PWM do motor ---- //

//Pinos para controle lógicco do motor 1 e 2
const int logic1m1 = 10;
const int logic2m1 = 11;
const int logic1m2 = 12;
const int logic2m2 = 13;

//Pinos que controlam a velocidade de cada motor
const int velocidade1 = 1;
const int velocidade2 = 2;

const int VBASE_CURVA = 100; //Velocidade base do motor, na qual adicionamos o ajuste PID
const int VBASE_RETA = 200; 
//Queremos que o nosso robô seja mais rápido nas retas, por isso a velocidade maior nas retas
//Os valores ideais só podem ser definidos com testes

//Armazena o valor PWM da velocidade 1, 2 e a velocidade base em cada caso
int velo1, velo2, veloBASE = 0;

// ---- Sensores Laterais --- //
const int dir = 8;
const int esq = 9;
const int CONTRASTE = 800; //Dado que serão um único sensor analógico, a leitura varia de 0 - 1023. Devemos definir um valor alto para o contraste ser preciso


// ----------------- Variáveis relativas ao PID ----------------- //

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
const int qt_esq = 5; //Define a quantidade de faixas da esquerda

int indc_dir = 0; //Armazena o indice do vetor de faixas da direita
int indc_esq = 0; //Armazena o indice do vetor de faixas da esquerda

int faixa_dir[qt_dir]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro - Indicará se a proxima leitura corresponde a inicio, cruzamento ou fim
int faixa_esq[qt_esq]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro - Indicará se a proxima leitura corresponde a curva ou reta
/*
 * OBS: A posição 0 destes vetores vão ser consideradas. O que significa que, a cada nova leitura, eu vou considerar um novo estado válido; e só depois de processar tudo que incrementa o índice 
 */
//DEFINIMOS:
//Para o vetor da esquerda
const int RETA = 0;
const int CURVA = 1;

//Para o vetor da direita
const int SEGUE = 1;      //A princípio não será usado, mas é bom deixar explícito
const int FIM = 0;
//const int CRUZAMENTO = 1 A princípio não preciso definir o cruzamento. Basta mandar o robô seguir enquanto não for o fim

// Kp Ki Kd ideais para reta e curva
double PID_reta[3] = {0, 0, 0};
double PID_curva[3] = {0, 0, 0};

//Variaveis de uso na loop
int direita; //Armazena a leitura atual da direita
int esquerda; //Armazena a leitura atual da esquerda
int parar = 0; //Aparentemente não há um jeito de desligar o arduino via código, por isso se esta variável for 1 o robô não fará nada
double ajuste;


// ------------------- PID ------------------- //
//Adaptado de: https://gist.github.com/ivanseidel/b1693a3be7bb38ff3b63

//Classe PID com todos os métodos e atributos necessários
class PID{
  
  private:
    double ajuste;
    uint16_t posicao;
    uint16_t posiAnt;
    double kP, kI, kD;      
    double P = 0, I = 0, D = 0;
    double pid;

    //Dada a propriedade do array de sensores, 2000 é o ponto de equilíbrio perfeito
    double setPoint = 2000;
    long tempoAnt = 0;  

  public:

  void setKPID(int estado)
  {
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
  
  double getPID()
  {
    //(for a white line, use readLineWhite() instead)    
    uint16_t posicao = qtr.readLineBlack(valorSensores); //Le os sensores e retorna um valor de 0 até 4000
    
    // Implementação PID
    ajuste = setPoint - posicao; //Dessa subtração, sabemos que o erro máximo é ?????? (não temos noção qual será o ajuste min-max, pois ele será a soma de P+I+D)
    float deltaTime = (millis() - tempoAnt) / 1000.0;
    tempoAnt = millis();
    
    //P
    P = ajuste * kP;
    
    //I
    I = I + (ajuste * kI) * deltaTime;
    
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
  pinMode(front1, INPUT);
  pinMode(front2, INPUT);
  pinMode(front3, INPUT);
  pinMode(front4, INPUT);
  pinMode(front5, INPUT);
  pinMode(esq, INPUT);
  pinMode(dir, INPUT); 

  // ------- Configurar barra de sensores ------- //
  //qtr.setTypeAnalog();
  qtr.setTypeRC();      // Não sei exatamente qual dos dois modos de operação
  qtr.setSensorPins((const uint8_t[]) {front1, front2, front3, front4, front5}, numSensores);  //5 Sensores - Valor flutua de 0 até 4000 - SetPoint será 2000
  calibraBarraSensores();

  // ----- Pinos do motor ----- //
  pinMode(logic1m1, OUTPUT);
  pinMode(logic2m1, OUTPUT);
  pinMode(logic1m2, OUTPUT);
  pinMode(logic2m2, OUTPUT);
  pinMode(velocidade1, OUTPUT);
  pinMode(velocidade2, OUTPUT);
  delay(100);
  digitalWrite(logic1m1, HIGH); //Já deixar setado apenas 1 sentido de rotação para os motores
  digitalWrite(logic2m1, LOW);
  digitalWrite(logic1m2, HIGH);
  digitalWrite(logic2m2, LOW);
  
  //analogWrite(velocidade1, 0);
  ledcWrite(velocidade1, 0);  //ledcWrite funciona?? É a melhor alternativa? Não sabemos...
  ledcWrite(velocidade2, 0);
}

void loop() 
{  
  if(parar)
    return;
    
  /* Evitar alocação no loop - poupa tempo */
  
  //Inicia a volta
  direita = analogRead(dir); //Retornam valores analógicos na range de 0 - 1023
  esquerda = analogRead(esq);
  
  //!!Problema importante: E se o programa entrar várias vezes nesses IFs ? Como filtrar? Colocar um intervalo de tempo mínimo de tolerância?
    //  Resposta: Só com testes saberemos....
    
  if(esquerda >= CONTRASTE)
  {
    if(faixa_esq[indc_esq] == RETA) //Estamos em uma reta
    {
      leitor.setKPID(RETA);
      veloBASE = VBASE_RETA;
    }
    else //Estamos em uma curva                        
    {
      leitor.setKPID(CURVA);
      veloBASE = VBASE_CURVA;
    }

    ajuste = leitor.getPID();

    //Dois valores BASE diferente - 1 pra reta 1 pra curva - O valor ideal só pode ser descoberto com testes.....
    velo1 = veloBASE + ajuste;
    velo2 = veloBASE - ajuste;

    velo1 = abs(velo1);
    velo2 = abs(velo2);
    
    if(velo1 > 255 && velo2 < 255)
    {
      ledcWrite(velocidade1, 255); 
      ledcWrite(velocidade2, velo2);
    }
    else if(velo2 > 255 && velo1 < 255)
    {
      ledcWrite(velocidade1, velo1); 
      ledcWrite(velocidade2, 255);
    }
    else if(velo1 > 255 && velo2 > 255)
    {
      ledcWrite(velocidade1, 255);
      ledcWrite(velocidade2, 255);
    }
    else
    {
      ledcWrite(velocidade1, velo1); 
      ledcWrite(velocidade2, velo2);
    } 
    
    indc_esq++;
  }

  //O programa DEVE testar os dois IFs, pois o vetor da direita deve estar devidamente atualizado para pararmos na hora certa
    //Será que durante um cruzamento ele consegue computar o da direita E o da esquerda??? Só com testes....
  if(direita >= CONTRASTE)
  {
    if(faixa_esq[indc_esq] == FIM) //Chegamos ao FIM do trajeto, devemos parar!!
    {
      parar = 1;
      
      ledcWrite(velocidade1, 0); //ledcWrite funciona?? É a melhor alternativa? Não sabemos...
      ledcWrite(velocidade2, 0);
    }
    
    //Não é o fim, basta incrementar o vetor e esperar pelo fim
    indc_dir++;
  }  
}
