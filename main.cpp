#include "functions.h"

int main(int argc, char** argv) {

    printf("\n\n\nIniciando SIMULADOR.\n\n");

    /* ----------------------------------------------
     * ABRE IMAGENS DE ENTRADA
     * ---------------------------------------------- */

    // -- verifica parametros de entrada
    if (argc != 3) {
        cout << "Forma de execucao: ./ws <arquivo_foto> <arquivo_textura>" << endl;
        cvWaitKey(0);
        return -1;
    }

    printf("Abrindo imagens...\n");

    // -- imagem ambiente
    string file_ambiente = argv[1];

    Mat im_ambiente = imread(file_ambiente);
    if (!im_ambiente.data) {
        cout << "Arquivo de imagem nao encontrado: " << file_ambiente << endl;
        return -1;
    }

    // -- imagem textura
    string file_textura = argv[2];

    Mat im_textura = imread(file_textura);
    if (!im_textura.data) {
        cout << "Arquivo de imagem nao encontrado: " << file_textura << endl;
        return -1;
    }


    /* ----------------------------------------------
     * EXECUCAO
     * --------------------------------------------- */

    // -- inicializacao
    printf("\n\nInicia processamento...\n\n");
    printf("<Pressione ESC para sair>\n\n");
    inicializa(im_ambiente, im_textura);

    // -- ciclo de execucao
    executa(0,0);
    while(1) if (cvWaitKey(1) == 27) break;
    destroyAllWindows();
    printf("\n\nPrograma finalizado.\n\n\n\n\n");

    /* ----------------------------------------------
     * FINALIZA PROGRAMA
     * ---------------------------------------------- */

    cvWaitKey(0);
    return 0;
}
