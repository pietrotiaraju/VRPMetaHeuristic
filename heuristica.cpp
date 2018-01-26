#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <chrono>
#include <random>
#include <tr1/random>
#define DIM 50
#define SPACE 100
#define NODES 32

//Definindo o tipo rota
typedef struct Route
{
    int k;
    int maxCapacity;
    int ID[NODES], coordX[NODES], coordY[NODES], demand[NODES], inertia[NODES], binaryAllocation[NODES];
    double prob[NODES], cumProb[NODES];
    int distanceMatrix[NODES][NODES];
    int currentSolution[DIM][SPACE];
} route;

//Bool list
bool checkFeasibility(int capacity, int demand[NODES], int solution[DIM][SPACE]);

//Double list
double randomUniform();

//Int list
int allocatedCustomers(int binary[NODES]);
int generateInitialNode(double random, double vec[NODES]);
int getNearestDistance(int id, int dist[NODES][NODES], int binary[NODES]);
int getNearestNode(int id, int dist[NODES][NODES], int binary[NODES]);
int hLength(int x, int matrix[DIM][SPACE]);
int vLength(int matrix[DIM][SPACE]);

//Void list
void calculateDistance(route *r);
void calculateInertia(route *r);
void calculateProb(route *r);
void cumulativeProb(route *r);
void emptyBinaryAllocation(route *r);
void emptyRoute(route *r);
void generateInitialSolution(route *r);
void importData(char dir[SPACE], route *r);

using namespace std;

main()
{
    route *master = new route;
    char path[SPACE] = "C:\\Users\\pietro\\Downloads\\A-VRP\\Instancias\\A32k5_C.txt";

    master->maxCapacity = 100;

    emptyRoute(master);
    importData(path, master);
    calculateDistance(master);
    generateInitialSolution(master);

    int i, j;

    for(i = 0; i <= NODES - 1; i++)
    {
        for(j = 0; j <= NODES - 1; j++)
        {
            cout << master->currentSolution[i][j] << " ";
        }

        cout << "\n";
    }

    cout << checkFeasibility(master->maxCapacity, master->demand, master->currentSolution);
}

//Importa a instancia
void importData(char dir[SPACE], route *r)
{
    int i = 0;
    FILE *conn = fopen(dir, "r");

    while(!feof(conn))
    {
        fscanf(conn, "%d %d %d %d", &(r->ID[i]), &(r->coordX[i]), &(r->coordY[i]), &(r->demand[i]));
        i++;
    }
}

//Zera matriz
void emptyRoute(route *r)
{
    int i, j;

    for(i = 0; i <= DIM - 1; i++)
    {
        for(j = 0; j <= SPACE - 1; j++)
        {
            r->currentSolution[i][j] = 0;
        }
    }
}

//Calcula o numero de rotas
int hLength(int x, int matrix[DIM][SPACE])
{
    int i;
    int sum = 0;

    for(i = 0; i <= SPACE - 1; i++)
    {
       if(matrix[x][i] != 0)
       {
           sum++;
       }
    }

    return(sum);
}

//Calcula o tamanho da rota
int vLength(int matrix[DIM][SPACE])
{
    int i;
    int sum = 0;

    for(i = 0; i <= DIM - 1; i++)
    {
       if(matrix[i][0] != 0)
       {
           sum++;
       }
    }

    return(sum);
}

//Calcula quantas posicoes alocadas no binary tem
int allocatedCustomers(int binary[NODES])
{
    int i;
    int sum = 0;

    for(i = 0; i <= NODES - 1; i++)
    {
        sum += binary[i];
    }

    return(sum);
}

void emptyBinaryAllocation(route *r)
{
    int i;

    for(i = 0; i <= NODES - 1; i++)
    {
        r->binaryAllocation[i] = 0;
    }
}

void calculateDistance(route *r)
{
    int i, j;

    for(i = 0; i <= NODES - 1; i++)
    {
        for(j = 0; j <= NODES - 1; j++)
        {
            r->distanceMatrix[i][j] = round(pow(pow(r->coordX[i] - r->coordX[j], 2) + pow(r->coordY[i] - r->coordY[j], 2), 0.5));
        }
    }
}

void calculateInertia(route *r)
{
    int i, j;

    //Nao ha inercia para o super source!!
    for(i = 0; i <= NODES - 1; i++)
    {
        //Verifica a disponibilidade do no
        if(r->binaryAllocation[i] == 0)
        {
            r->inertia[i] = (r->distanceMatrix[i][0])*(r->demand[i]);
        }
        else
        {
            r->inertia[i] = 0;
        }
    }
}

void calculateProb(route *r)
{
    int i;
    double sum = 0;

    //Acumula a inercia
    for(i = 0; i <= NODES - 1; i++)
    {
        sum += r->inertia[i];
    }

    //Razao entre a participacao da inercia no total
    for(i = 0; i <= NODES - 1; i++)
    {
        r->prob[i] = r->inertia[i]/sum;
    }
}

double randomUniform()
{
  unsigned seed = chrono::system_clock::now().time_since_epoch().count();

  default_random_engine generator(seed);
  uniform_real_distribution<double> distribution(0,1);

  return(distribution(generator));
}

void cumulativeProb(route *r)
{
    int i;
    double result[NODES];

    partial_sum(r->prob, r->prob + NODES, result);

    for(i = 0; i <= NODES - 1; i++)
    {
        r->cumProb[i] = result[i];
    }
}

//Gera o no inicio a partir de um numero aleatorio e a distribuicao
int generateInitialNode(double random, double cumprob[NODES])
{
    int i, initialNode;

    for(i = 0; i <= NODES - 2; i++)
    {
        if(random >= cumprob[i] && random < cumprob[i+1])
        {
            initialNode = i + 2;
        }
    }

    return(initialNode);
}

int getNearestNode(int id, int dist[NODES][NODES], int binary[NODES])
{
    int i, minID, node;

    minID = 1000000;

    if(allocatedCustomers(binary) <= NODES - 1)
    {
        for(i = 0; i <= NODES - 1; i++)
        {
            if(dist[id-1][i] < minID && dist[id-1][i] > 0 && binary[i] == 0)
            {
                minID = dist[id-1][i];
                node = i + 1;
            }
        }
    }
    else
    {
        node = 0;
    }

    return(node);
}

int getNearestDistance(int id, int dist[NODES][NODES], int binary[NODES])
{
    int i, minID, node;

    minID = 1000000;

    if(allocatedCustomers(binary) <= NODES - 2)
    {
        for(i = 0; i <= NODES - 1; i++)
        {
            if(dist[id-1][i] < minID && dist[id-1][i] > 0 && binary[i] == 0)
            {
                minID = dist[id-1][i];
                node = i + 1;
            }
        }
    }
    else
    {
        node = 0;
    }

    return(minID);
}

void generateInitialSolution(route *r)
{
    //Seta os contadores para zero e inicia o procedimento de construcao
    int i = 0, j = 1, k;
    int randomNode, nextNode, dist1, dist2;
    int currentCapacity = r->maxCapacity;

    emptyBinaryAllocation(r);
    r->binaryAllocation[0] = 1;

    while(allocatedCustomers(r->binaryAllocation) <= NODES - 1)
    {
        //Atualiza os dados
        calculateInertia(r);
        calculateProb(r);
        cumulativeProb(r);

        //Atualiza parametros
        r->currentSolution[i][0] = 1;

        while(true)
        {
            //Se nao tiver nenhum no ainda alocado na rota
            if(hLength(i, r->currentSolution) == 1)
            {
                //Sorteia o no
                randomNode = generateInitialNode(randomUniform(), r->cumProb);

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[randomNode-1];

                if(currentCapacity >= 0)
                {
                    //Pluga o no sorteado na proxima posicao da rota
                    r->currentSolution[i][j] = randomNode;
                    //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                    r->binaryAllocation[randomNode-1] = 1;
                    //Adianta o contador
                    j++;
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }
            //Se so tiver o no inicial alocado na rota
            if(hLength(i, r->currentSolution) == 2)
            {
                //Pega o no mais proximo do anteriormente sorteado
                nextNode = getNearestNode(randomNode, r->distanceMatrix, r->binaryAllocation);

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[nextNode-1];

                if(currentCapacity >= 0)
                {
                    //Pluga o no sorteado na proxima posicao da rota
                    r->currentSolution[i][j] = nextNode;
                    //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                    r->binaryAllocation[nextNode-1] = 1;
                    //Adianta o contador
                    j++;
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }

            if(hLength(i, r->currentSolution) >= 3)
            {
                //Pega o no mais proximo do anteriormente sorteado
                dist1 = getNearestDistance(r->currentSolution[i][1], r->distanceMatrix, r->binaryAllocation);
                dist2 = getNearestDistance(r->currentSolution[i][hLength(i, r->currentSolution) - 1], r->distanceMatrix, r->binaryAllocation);

                if(dist1 >= dist2)
                {
                    nextNode = getNearestNode(r->currentSolution[i][1], r->distanceMatrix, r->binaryAllocation);
                }
                else
                {
                    nextNode = getNearestNode(r->currentSolution[i][hLength(i, r->currentSolution) - 1], r->distanceMatrix, r->binaryAllocation);
                }

                //Tira da capacidade do veiculo a demanda do cliente
                currentCapacity -= r->demand[nextNode-1];

                if(currentCapacity >= 0)
                {
                    if(dist1 >= dist2)
                    {
                        //Pluga o no sorteado na proxima posicao da rota
                        r->currentSolution[i][hLength(i, r->currentSolution)] = nextNode;
                        //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                        r->binaryAllocation[nextNode-1] = 1;
                    }
                    else
                    {
                        //Desloca o vetor solucao para a direita para pode inserir o novo no
                        for(k = hLength(i, r->currentSolution) - 1; k >= 1; k--)
                        {
                            r->currentSolution[i][k+1] = r->currentSolution[i][k];
                        }

                        //Pluga o no sorteado no inicio da rota
                        r->currentSolution[i][1] = nextNode;
                        //Muda a alocacao binaria de 0 para 1 utilizada pelo no inicial
                        r->binaryAllocation[nextNode-1] = 1;
                    }
                }
                else
                {
                    //Pluga o super source no final da rota
                    r->currentSolution[i][hLength(i, r->currentSolution)] = 1;
                    //Sai do laco
                    break;
                }
            }
        }

        //Passa pra proxima rota
        i++;
        //Primeira posicao na proxima rota
        j = 1;
        //Repoe a capacidade do veiculo
        currentCapacity = r->maxCapacity;
    }
}

bool checkFeasibility(int capacity, int demand[NODES], int solution[DIM][SPACE])
{
    int i, j;
    int testCapacity[vLength(solution)];
    int check[vLength(solution)];

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        testCapacity[i] = capacity;
    }

    for(i = 0; i <= vLength(solution) - 1; i++)
    {
        for(j = 1; j <= hLength(i, solution) - 1; j++)
        {
            testCapacity[i] = testCapacity[i] - demand[solution[i][j]-1];
        }
         if(testCapacity[i] < 0)
        {
            return(false);
        }
    }

    return(true);
}
