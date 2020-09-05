// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C"
{
#include "remoteApi/extApi.h"
}

#include <iostream>
#include <string.h>

using namespace std;

// Classe PID
class PID{
  
  private:
    double ajuste;
    uint16_t posicao;
    uint16_t posiAnt;
    double kP, kI, kD;      
    double P = 0,D = 0,I = 0;
    double pid;
    double deltaTime;
    double soma;

    //Dada a propriedade do array de sensores, 2500 é o ponto de equilíbrio perfeito
    double setPoint = 2500;
    double tempoAnt = 0;  

  public:
  
  PID(double Pi, double Ii, double De)
  {
    kP = Pi;
    kI = Ii;
    kD = De;
  }

  double getPID(int v[4])
  {
    if(v[0] + v[1] + v[2] + v[3] + v[4] == 0)
      v[0] = 1;
    
    uint16_t posicao = (1000*v[0] + 2000*v[1] + 3000*v[2] + 4000*v[3] + 5000*v[4]) /(v[0] + v[1] + v[2] + v[3] + v[4]);

    // Implementação PID
    ajuste = setPoint - posicao; //Dessa subtração, sabemos que o erro máximo é ?????? (não temos noção qual será o ajuste min-max, pois ele será a soma de P+I+D)
    
    deltaTime = (double) (clock() - tempoAnt) * 1000 / CLOCKS_PER_SEC;   //Ficar esperto com este deltaTime
    tempoAnt = clock();    
    
    soma +=deltaTime;
    //P
    P = ajuste * kP;
    
    //I
    I = I + (ajuste * kI) * deltaTime;
    
    //D
    D = (posiAnt - posicao) * kD / deltaTime;
    posiAnt = posicao;
    
    // Soma tudo
    double val = P + I + D;
    return val;
  }
};

PID pid(0.0011, 0.000003, 0.000008); 

int main(int argc, char **argv)
{
  //Variaveis para conexao do servidor
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  //Handles dos  motores
  int leftMotorHandle = 0;
  float vLeft = 0;
  int rightMotorHandle = 0;
  double vRight = 0;
  //Handles e nomes dos sensores
  string sensorNome[5] = {"LeftSensor", "MiddleSensor", "RightSensor", "LeftSensor2", "RightSensor2"};
  int sensorHandle[5];

  int res[2];
  simxUChar* image;
  int sensorResponse[5] = {0,0,0,0,0};

  //Tenta estabelecer conexao com a simulacao (nao esqueca de dar play)
  int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

  //Variáveis do seguidor
  int v[4];
  double veloBASE = 10;
  double ajuste;


  //Se a conexao e estabelicida, sera retornado um valo diferente de 0
  if (clientID != -1)
  {
    printf("Servidor conectado!\n");
    
    // inicialização dos motores
    if (simxGetObjectHandle(clientID, (const simxChar *)"DynamicLeftJoint", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor esquerdo nao encontrado!\n");
    else
      printf("Conectado ao motor esquerdo!\n");

    if (simxGetObjectHandle(clientID, (const simxChar *)"DynamicRightJoint", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      printf("Handle do motor direito nao encontrado!\n");
    else
      printf("Conectado ao motor direito!\n");

    
    // inicialização dos sensores (remoteApi)
    for (int i = 0; i < 5; i++)
    {
      if (simxGetObjectHandle(clientID, (const simxChar *)sensorNome[i].c_str(), (simxInt *)&sensorHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
      {
        printf("Handle do sensor %s nao encontrado!\n",sensorNome[i].c_str());
      }
      else
      {
        printf("Conectado ao sensor %s \n",sensorNome[i].c_str());
        simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming);
      }
    }
  
    // desvio e velocidade do robô
    while (simxGetConnectionId(clientID) != -1) // enquanto a simulação estiver ativa
    {
      for (int i = 0; i < 5; i++)
      {
        if(simxGetVisionSensorImage(clientID,sensorHandle[i],res,&image,0,simx_opmode_streaming) == simx_return_ok)
        {
          int sum = 0;
          for (int i = 0; i < res[0]*res[0]; i++)
          {
            sum += (int) image[i];
          }
          sum = sum/(res[0]*res[0]); //Média dos valores preto-branco dos pixels da imagem
                                     //Podemos simular esse valor como um sensor IR
          v[i] = sum;
          
          if(sum > 60)
            sensorResponse[i] = 1;
          else
            sensorResponse[i] = 0;
          
          printf("Sensor[%i] : %d \n", i,sensorResponse[i]);

    
        }
      }
      //Com as leituras feitas
      ajuste = pid.getPID(v);

      //Dois valores BASE diferente - 1 pra reta 1 pra curva - O valor ideal só pode ser descoberto com testes.....
      vLeft = veloBASE - ajuste;
      vRight = veloBASE + ajuste;

      printf("\nTemos v1: %f e v2: %f\n", vLeft, vRight);
      
      // atualiza velocidades dos motores
      simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
      simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
      
      extApi_sleepMs(5);
    }
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else
  {
    printf("Problemas para conectar o servidor!\n");
  }
  return 0;
}
