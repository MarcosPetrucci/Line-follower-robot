
// ------- DECLARACAO DE VARIAVEIS ------- //

// ---- Sinal PWM do motor ---- //
const int motor1 = 1;
const int motor2 = 2;

// ---- Sensores frontais --- //
const int front1 = 3;
const int front2 = 4;
const int front3 = 5;
const int front4 = 6;
const int front5 = 7;

// ---- Sensores Laterais --- //
const int dir = 8;
const int esq = 9;

// ----- Variáveis relativas ao PID ----- //

//!DEFINIMOS: 
//  0 para reta
//  1 para curva
int estado = 0;

const int qt_dir = 5; //Define a quantidade de faixas da direita
const int qt_esq  =5; //Define a quantidade de faixas da esquerda

int indc_dir = 0; //Armazena o indice do vetor de faixas da direita
int indc_esq = 0; //Armazena o indice do vetor de faixas da esquerda

int faixa_dir[qt_dir]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro
int faixa_esq[qt_esq]; //Deve ser preenchido manualmente ou automáticamente via magnetômetro



// Kp Ki Kd ideais para reta e curva
double PID_reta[3] = {0, 0, 0};
double PID_curva[3] = {0, 0, 0};

// --------- PID ---------//
//Creditos: Ivan Seidel
//Inspirado de: https://gist.github.com/ivanseidel/b1693a3be7bb38ff3b63

class PID{
  
  private:
    double erro;
    double valor;
    double valorAnt;
    double kP, kI, kD;      
    double P, I, D;
    double pid;
  
    double setPoint;
    long tempoAnt;  

  public:
  void setEquilibrio(double _setPoint){
    setPoint = _setPoint;
  }

  //Como ler todos os sensores?
  //Será uma sensorBar? --> melhor opção!!!
  void addLeitura(double s1, double s2, double s3, double s4, double s5){
    valor = _valor;
  }

  void setKPID()
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
    //! Implementar para a minha finalidade //
    
    // Implementação PID
    erro = setPoint - valor;
    float deltaTime = (millis() - tempoAnt) / 1000.0;
    tempoAnt = millis();
    
    //P
    P = erro * kP;
    
    //I
    I = I + (erro * kI) * deltaTime;
    
    //D
    D = (valorAnt - valor) * kD / deltaTime;
    valorAnt = valor;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};


void setup() {
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

  
}

void loop() {
 
  PID leitor;
  leitor.setKPID();
}
