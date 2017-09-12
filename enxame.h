#ifndef _ENXAME_H_
#define _ENXAME_H_

#include <vector>
#include <algorithm>
#include "particula.h"

using namespace std;

class Enxame {
public:
    Enxame(PsoConfig config) {
        this->config = config;
        qualidade = 999999;
		for (int i = 0; i < config.tamanhoEnxame; ++i) {
			Particula p(config);
			populacao.push_back(p);
			float q = p.getQualidade();
			if (qualidade > q) {
				qualidade = q;
				lBest = p.getPBest();
			}
		}
	}

    void imprime() {
        for (int i =0; i < populacao.size(); ++i) {
            populacao[i].imprime();
			cout << "-> qualidade pBest: " << (int)(populacao[i].getQualidade()) << endl << endl;
		}
		cout << "*****************************************" << endl;
		cout << "=> qualidade lBest: " << (int)qualidade << endl << endl;
		cout << "*****************************************" << endl;
		cin.get();
    }

	void atualizaEnxame() {
		for (int i = 0; i < config.tamanhoEnxame; i++) {
			populacao[i].atualizaVelocidade(lBest);
			float q = populacao[i].atualizaVetor();
			if (qualidade > q) {
				qualidade = q;
				lBest = populacao[i].getPBest();
			}
		}
	}

	void ordenaEnxame() {
        //sort(populacao.begin(), populacao.end());
	}

	float getQualidade() {
		return qualidade;
	}

    vector<float> getLBest() {
        return lBest;
    }

private:
    vector<Particula> populacao;
	vector<float> lBest;
	float qualidade;
	PsoConfig config;
};

#endif // ENXAME

