#include "pso.h"

vector<float> Pso::executa(PsoConfig config) {

    clock_t tStart = clock();

    // 1. Inicializa a populacao
    Enxame enxame(config);
    //enxame.imprime();

    // 2. Para i=0 ate tamanho do enxame
    //   2.1. inicia Xi com uma solucao aleatoria
    //   2.2. inicia Vi com uma vel. aleatoria < Vmax
    //   2.3. Pbest_i <- Lbest_i <- Gbest_i
    int pqt_iteracoes = 0;
    double tempo_exec = 0;
    float qualidade = INT_MAX;

    // 3. Enquanto nao atingir condicao de parada
    //   3.1. Para i=0 ate tamanho do enxame:
    //     3.1.1. atualiza Vi
    //     3.1.2. Xi <- Xi + Vi
    //     3.1.3. Pbest_i <- melhor entre Xi e Pbest_i
    //     3.1.4. Lbest_i <- melhor entre Pbest_i e Lbest_i
    while ((pqt_iteracoes < config.maxIteracoes) &&
           (qualidade > config.minQualidade) &&
           (tempo_exec < config.maxCongelamento))
    {
        for (int i = 0; i < config.tamanhoEnxame; i++) {
			enxame.atualizaEnxame();
		}
        //enxame.imprime();
        pqt_iteracoes++;
        qualidade = enxame.getQualidade();
        tempo_exec = (double)(clock() - tStart)/CLOCKS_PER_SEC;
    }

    // 4. Retorna melhor Lbest
    printf("Fim da execucao do PSO.\n");
    printf("Tempo exe: %.2fs\n", tempo_exec);
    printf("Qualidade: %.2f\n", qualidade);
    printf("Iteracoes: %d\n", pqt_iteracoes);
    return enxame.getLBest();
}
