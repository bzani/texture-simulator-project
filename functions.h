#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "libs.h"
#include "pso.h"

/* ##############################################
 *
 * DEFINICOES
 *
 * ##############################################
 */

// -- definitions
#define MAX_BYTE 255
#define MAX_PIC_SCALE 150000
#define MAX_TEX_SCALE 10000
#define MIN_SURFACE_SIZE 300
#define MIN_CONTOUR_SIZE 300
#define TAM_MASK 3

// -- threshold limiters
#define MAX_BLUR 20
#define MAX_CANNY 255
#define MAX_POLYG 15
#define MAX_MORFO 9
#define MAX_CONTR 30
#define MAX_BRILHO 100
#define MAX_GAMMA 100
#define MAX_BINAR 7
#define MAX_HSV 510
#define MAX_TRANSF 100
#define MAX_ILUMN 50

// -- pso rules
#define MAX_ERRO 9999999
#define QLD_ACEIT 7000
#define RULE_LOW  1
#define RULE_MED  15
#define RULE_HIGH 100

// -- enumeraveis
enum E_FLUXO {
    EF_AJUSTE,
    EF_TEXTURA,
    EF_MANUAL,
    EF_PSO,
    EF_RESULT
};

enum Q_BEST {
    Q_PSO,
    Q_HULL,
    Q_SIMPL
};


/* ##############################################
 *
 * VARIAVEIS GLOBAIS
 *
 * ##############################################
 */

// -- I/O global
ofstream log_file("log.txt", ios::app);

// -- flags
int pdi_trace=0;
int st_result=0;

// -- imagens globais
Mat im_source;      //imagem original em escala reduzida
Mat im_texture;     //imagem original da textura
Mat im_wallpaper;   //imagem preenchida com textura lado-a-lado
Mat im_work;        //imagem de trabalho (fluxo)
Mat im_selarea;     //guarda area de aplicacao da textura (clique)
Mat im_selbbox;     //guarda bounding box da area de aplicacao
Mat im_selhull;     //guarda area de contorno fechado
Mat im_seltran;     //guarda area de transformacao destino
Mat im_selrpso;     //guarda area de transformacao destino (PSO)
Mat im_seltman;     //guarda area de transformacao destino (manual)
Mat im_selfina;     //guarda area resultado
Mat im_areatxt;     //textura aplicada a area do b.box
Mat im_transfm;     //area papel de parede transformada
Mat im_trsfhul;     //area papel de parede transformada (c.hull)
Mat im_trsfpso;     //area papel de parede transformada (pso)
Mat im_trsftxr;     //area papel de parede transformada (pso)
Mat im_hull;        //contornos em fecho convexo
Mat im_ajuste;      //pos ajustes
Mat im_final;       //imagem resultante final
Size src_size;      //guarda dimensoes da imagem original (foto)

// -- elementos de trackbar
int thresh_blur = 2;
int thresh_canny1 = 11;
int thresh_canny2 = 30;
int thresh_contrs = 10;
int thresh_brilho = 0;
int thresh_dilation = 3;
int thresh_abfe = 0;
int thresh_histog = 0;
int thresh_hiscor = 0;
int thresh_apxpoly = 1;
int thresh_gamma = 10;
int thresh_sharp = 0;
int thresh_binar = 1;
int thresh_binblk = 12;
int thresh_binc = 1;
int thresh_ilumn = 15;

// hsv
int thresh_hsv[3]={255,255,255};

// -- demais parametros

// tipo do elemento estrutural
//   0 = MORPH_RECT
//   1 = MORPH_CROSS
//   2 = MORPH_ELLIPSE
int dilation_elem = 2;
int erosion_elem = 2;

// tipo do elemento estrutural
int const max_elem = 2;
int const max_kernel_size = 21;

// -- contornos
vector< vector<Point> > contours;
vector<Vec4i> hierarchy;
vector< vector<Point> > cHull;
vector<Rect> bRect;
int selected=0;

// -- eventos do mouse
Point mouse_click, mouse_rls;

// -- variaveis da transformacao
Rect selRect;
vector<Point> selHull;
vector<Point> selCont;
vector<Point> ptsOrig;
vector<Point> ptsDest;
Point2f inputQuad[4];
Point2f outputQuad[4];

// -- flags e variaveis de estado
bool mouse_st=0;
bool mp_transfd=0;
int warp_st=0;
int flux_st=0;

// -- nome das janelas
const char* win_tbar_ajuste = "Ajuste";
const char* win_tbar_proces = "Processamento";
const char* win_tbar_binar = "Binaria";
const char* win_tbar_morfo = "Morfologia";
const char* win_tbar_hsv = "HSV";
const char* win_tbar_contor = "Contornos";
const char* win_tbar_cvhull = "Fecho Convexo";
const char* win_tbar_bdnbox = "Bounding Box";
const char* win_main_source = "Original";
const char* win_main_wallpaper = "Textura Orig.";
const char* win_main_final = "Resultado";
const char* win_final_pre = "Sem Transformacao";
const char* win_final_transf = "Transformacao Auto/Manual";
const char* win_final_pso = "Transformacao PSO";
const char* win_area_cont = "Area Contorno";
const char* win_area_bbox = "Area BBox";
const char* win_area_hull = "Area Fecho";
const char* win_area_tran = "Area Transf.";
const char* win_area_pso = "Area PSO";
const char* win_tran_manual = "Transformada Manual";
const char* win_tran_auto = "Transformada Auto";
const char* win_tran_pso = "Transformada PSO";
const char* win_text_bbox = "Textura Bbox";

/* ##############################################
 *
 * FUNCOES (header)
 *
 * ##############################################
 */

/*
 * ----------------------------------------------
 * VARIADAS
 * ----------------------------------------------
 */

void cria_track_bars();
const char* itocs(int n);
string itos(int n);
vector<Point> pf2vp(Point2f *pf, int t);
Point2f* vp2pf(vector<Point> vp, int t);

/*
 * ----------------------------------------------
 * ESCALA
 * ----------------------------------------------
 */

Mat reduz_resolucao(Mat img);
Mat reduz_textura(Mat img);

/*
 * ----------------------------------------------
 * INTELIGENCIA COMPUTACIONAL - PSO
 * ----------------------------------------------
 */

Mat cria_figura(Size size, vector<Point> points);
Mat cria_poligono(vector<Point> points);
float calc_area_contorno_img(Mat img);
int calcula_dist_eucl(Point a, Point b);
inline double prod_cruzado_z(const Point &a, const Point &b);
inline double verif_orientacao(const Point &a, const Point &b, const Point &c);
void ordena_4pontos_cw(Point points[4]);
vector<Point> ordena_pontos_ccw(vector<Point> vp);
void transform_espacial_pso();
void aplica_textura_pso();
float calc_qualid_area(Mat res);
float funcao_objetivo(vector<float> &solucao);
void executa_pso();

/*
 * ----------------------------------------------
 * MORFOLOGIA
 * ----------------------------------------------
 */

Mat oper_and(Mat ent1, Mat ent2);
Mat oper_or(Mat ent1, Mat ent2);
Mat morf_erosion(Mat im_area, int es);
void morf_erosion();
Mat morf_dilation(Mat im_area, int es);
void morf_dilation();
void morf_abertura();
void morf_fechamento();
Mat morf_fronteira(Mat im_area);

/*
 * ----------------------------------------------
 * OPERACOES PRINCIPAIS
 * ----------------------------------------------
 */

int calcula_dist_eucl(Point2f a, Point2f b);
void controleHSV();
void converte_cinza();
void plot_histograma(const char* win_name);
void equaliza_histograma_pre();
void equaliza_histograma_pos();
void equaliza_ilum_textr();
void suaviza_bordas_textr();
void ajusta_brilho_contraste();
void correcao_gamma();
void ajusta_nitidez();
void aplica_median_blur();
void aplica_filtros();

/*
 * ----------------------------------------------
 * OPERACOES EM IMAGENS BINARIAS
 * ----------------------------------------------
 */

void binariza_imagem();
void binar_adaptiva_auto();
void binarizacao_adaptiva();
void inverte_imagem();
void aplica_canny();
void desenha_margem();
void aplica_morfologias();
void busca_contornos();

/*
 * ----------------------------------------------
 * TEXTURA
 * ----------------------------------------------
 */

void valida_area_aplic();
void texturiza_area_bbox();
void define_pontos_orig();
void define_pontos_dest();
void cria_fundo_textura();
void aplica_textura();

/*
 * ----------------------------------------------
 * MAPEAMENTO/TRANSFORMACAO ESPACIAL
 * ----------------------------------------------
 */

void transformacao_espacial();
void calc_melhor_transf();

/*
 * ----------------------------------------------
 * FUNCOES DE CALLBACK
 * ----------------------------------------------
 */
void callback_transform(int e, int x, int y, int, void*);
void callback_executa(int event, int x, int y, int flags, void* userdata);

/*
 * ----------------------------------------------
 * FLUXO PRINCIPAL / EVENTOS
 * ----------------------------------------------
 */

void seta_fluxo(int new_st);
void display_info();
void grava_controles(int,void*);
void inicializa(Mat im_amb, Mat im_tex);
void executa(int,void*);

/* ##############################################
 *
 * FUNCOES (implem.)
 *
 * ##############################################
 */

/*
 * ----------------------------------------------
 * VARIADAS
 * ----------------------------------------------
 */

/// cria as trackbars para realizar os testes manuais,
/// da etapa de pre-processamento e segmentacao
void cria_track_bars()
{
    // -- trackbars do controle HSV
    namedWindow(win_tbar_hsv, CV_WINDOW_AUTOSIZE);
    createTrackbar("H", win_tbar_hsv, &thresh_hsv[0], MAX_HSV, executa);
    createTrackbar("S", win_tbar_hsv, &thresh_hsv[1], MAX_HSV, executa);
    createTrackbar("V", win_tbar_hsv, &thresh_hsv[2], MAX_HSV, executa);

    // -- trackbars do ajuste
    namedWindow(win_tbar_ajuste, CV_WINDOW_AUTOSIZE);
    createTrackbar("Blur", win_tbar_ajuste, &thresh_blur, MAX_BLUR, executa);
    createTrackbar("Hist.Pre", win_tbar_ajuste, &thresh_histog, 1, executa);
    createTrackbar("Contraste", win_tbar_ajuste, &thresh_contrs, MAX_CONTR, executa);
    createTrackbar("Brilho", win_tbar_ajuste, &thresh_brilho, MAX_BRILHO, executa);
    createTrackbar("Gamma", win_tbar_ajuste, &thresh_gamma, MAX_GAMMA, executa);
    createTrackbar("Sharp", win_tbar_ajuste, &thresh_sharp, 1, executa);

    // -- trackbars do canny
    namedWindow(win_tbar_proces, CV_WINDOW_AUTOSIZE);
    createTrackbar("Canny-1", win_tbar_proces, &thresh_canny1, MAX_CANNY, executa);
    createTrackbar("Canny-2", win_tbar_proces, &thresh_canny2, MAX_CANNY, executa);

    // -- trackbars da morfologia
    namedWindow(win_tbar_morfo, CV_WINDOW_AUTOSIZE);
    const char* morf_types = "Tipo:0F 1A 2FF 3AA 4FA 5AF 6FE 7FD 8AE 9AD)";
    createTrackbar(morf_types, win_tbar_morfo, &thresh_abfe, MAX_MORFO, executa);
    createTrackbar("Dilatacao/Erosao", win_tbar_morfo, &thresh_dilation, max_kernel_size, executa);

    // -- trackbars dos contornos
    namedWindow(win_tbar_contor, CV_WINDOW_AUTOSIZE);
    createTrackbar("Ap.Polyg.", win_tbar_contor, &thresh_apxpoly, MAX_POLYG, executa);
}

/// converte int para const char
const char* itocs(int n)
{
    stringstream ss;
    ss << n;
    return ss.str().c_str();
}

/// converte int para string
string itos(int n)
{
    stringstream ss;
    ss << n;
    return ss.str();
}

/// converte vetor de pontos float para int
vector<Point> pf2vp(Point2f *pf, int t)
{
    vector<Point> vp;
    for(int i=0;i<t;i++) vp.push_back(pf[i]);
    return vp;
}

/// converte vetor de pontos int para float
Point2f* vp2pf(vector<Point> vp, int t)
{
    Point2f pf[t];
    for(int i=0;i<t;i++) pf[i]=vp[i];
    return pf;
}


/*
 * ----------------------------------------------
 * ESCALA
 * ----------------------------------------------
 */

/// reduz escala da imagem original para tratamento
Mat reduz_resolucao(Mat img)
{
    // -- detecta melhor escala
    int razao=0;
    int x = img.cols;
    int y = img.rows;
    while ((x*y) > MAX_PIC_SCALE)
    {
        razao+=2;
        x = (int)(img.cols/razao);
        y = (int)(img.rows/razao);
    }
    Mat mr;
    resize(img,mr,Size(x,y));

    // -- informacoes de redimensionamento
    printf("Razao de escala p/ reducao (ambient): %d\n", razao);
    printf("Tamanho da imagem original: %d x %d\n", img.cols, img.rows);
    printf("Tamanho da imagem reduzida: %d x %d\n", mr.cols, mr.rows);

    // -- retorna imagem reduzida
    src_size = mr.size();
    imshow(win_main_source, mr);
    return mr;
}

/// reduz escala da textura original para tratamento
Mat reduz_textura(Mat img)
{
    // -- detecta melhor escala
    int razao=0;
    int x = img.cols;
    int y = img.rows;
    while ((x*y) > MAX_TEX_SCALE)
    {
        razao+=2;
        x = (int)(img.cols/razao);
        y = (int)(img.rows/razao);
    }
    Mat mr;
    resize(img,mr,Size(x,y));

    // -- informacoes de redimensionamento
    printf("Razao de escala p/ reducao (textura): %d\n", razao);
    printf("Tamanho da textura original: %d x %d\n", img.rows, img.cols);
    printf("Tamanho da textura reduzida: %d x %d\n", mr.rows, mr.cols);

    // -- retorna imagem reduzida
    imshow(win_main_wallpaper, mr);
    return mr;
}


/*
 * ----------------------------------------------
 * INTELIGENCIA COMPUTACIONAL - PSO
 * ----------------------------------------------
 */

/// cria um poligono preenchido utilizando conceito de
/// fecho convexo e contornos, a partir de tamanho da
/// imagem e um vetor de pontos
Mat cria_figura(Size size, vector<Point> points)
{
    Mat img(size, CV_8UC1, Scalar(0));

    // -- desenha fecho convexo
    vector<int> hull;
    convexHull(Mat(points), hull, true);

    // -- liga os pontos com linhas
    int hullcount = (int)hull.size();
    Point pt0 = points[hull[hullcount-1]];
    for(int i = 0; i < hullcount; i++ )
    {
        Point pt = points[hull[i]];
        line(img, pt0, pt, Scalar(255), 1, LINE_4);
        pt0 = pt;
    }

    // -- preenche fecho convexo e guarda contorno
    vector< vector<Point> > c1;
    vector<Vec4i> hi;
    findContours(img, c1, hi, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    for( int i = 0; i< (int)c1.size(); i++ )
    {
        drawContours( img, c1, i, Scalar(255), CV_FILLED, 8, hi, 0, Point() );
    }

    // -- retorna imagem criada
    return img;
}

/// cria um poligono preenchido a partir de vetor de pontos
Mat cria_poligono(vector<Point> points)
{
    Mat img(src_size,CV_8UC1,Scalar(0));
    vector<Point> p = ordena_pontos_ccw(points);
    fillConvexPoly(img,p,Scalar(255));
    return img;
}

/// calcula a area interior de um contorno
float calc_area_contorno_img(Mat img)
{
    vector< vector<Point> > c1;
    vector<Vec4i> hi;
    findContours(img, c1, hi, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    float area=0;
    for(int i=0; i<c1.size(); i++)
        area+=contourArea(c1[i]);
    return area;
}

/// calcula distancia euclidiana entre 2 pontos
int calcula_dist_eucl(Point a, Point b)
{
    return (int)(norm(a-b));
}

/// retorna produto cruzado de 2 pontos
inline double prod_cruzado_z(const Point &a, const Point &b)
{
    // -- retorna o componente "z" do produto cruzado de "a" e "b"
    return a.x * b.y - a.y * b.x;
}

/// verifica orientacao de uma ordem de pontos
inline double verif_orientacao(const Point &a, const Point &b, const Point &c)
{
    // -- a orientacao eh
    // -- (+) se abc esta em sentido anti-horario,
    // -- (-) caso contrario equivale a 2x a area do triangulo abc, calculado
    // -- pela formula Shoelace
    return prod_cruzado_z(a, b) + prod_cruzado_z(b, c) + prod_cruzado_z(c, a);
}

/// metodo Shoelace para ordenacao de 4 pontos
void ordena_4pontos_cw(Point points[4])
{
    Point& a = points[0];
    Point& b = points[1];
    Point& c = points[2];
    Point& d = points[3];

    if (verif_orientacao(a, b, c) < 0.0)
    {
        // -- triang. abc esta no sentido horario, encaixar "d"
        if (verif_orientacao(a, c, d) < 0.0)
        {
            return;         // ok
        } else if (verif_orientacao(a, b, d) < 0.0)
        {
            std::swap(d, c);
        } else {
            std::swap(a, d);
        }
    } else if (verif_orientacao(a, c, d) < 0.0)
    {
        // -- triang. abc esta no sentido anti-hor. e acb sent.hor.
        // -- entao, acd esta no sentido horario
        if (verif_orientacao(a, b, d) < 0.0)
        {
            std::swap(b, c);
        } else {
            std::swap(a, b);
        }
    } else {
        // -- triangulo abc esta no sent. anti-hor. e acd tb
        // -- entao, abcd esta no sentido anti-hor.
        std::swap(a, c);
    }
}

/// ordena 4 pontos em sentido antihorario
vector<Point> ordena_pontos_ccw(vector<Point> vp)
{
    Point points[4];
    vector<Point> saida(4);
    // -- utiliza metodo Shoelace para ordenacao
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            if(j == i)  continue;
            for(int k = 0; k < 4; k++)
            {
                if(j == k || i == k) continue;
                for(int l = 0; l < 4; l++)
                {
                    if(j == l || i == l || k == l) continue;
                    points[0] = vp[i];
                    points[1] = vp[j];
                    points[2] = vp[k];
                    points[3] = vp[l];
                    ordena_4pontos_cw(points);
                }
            }
        }
    }
    // -- decide que o primeiro ponto eh o mais prox. da origem
    int ds=INT_MAX,k=0;
    for (int i=0; i<4; i++)
    {
        int de = calcula_dist_eucl(ptsOrig[0], points[i]);
        if (de < ds)
        {
            saida[0]=points[i];
            ds=de;
            k=i;
        }
    }
    for (int i=1; i<4;i++)
    {
        saida[i]=points[(i+k)%4];
    }
    return saida;
}

/// realiza transformacao perspectiva apos executar o PSO
void transform_espacial_pso()
{
    for(int k=0; k<4; k++)
    {
        inputQuad[k] = ptsOrig[k];
        outputQuad[k] = ptsDest[k];
    }
    Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(im_areatxt, im_transfm, lambda, src_size);
    if (pdi_trace) imshow(win_tran_pso, im_transfm);
}

/// aplica textura na imagem resultante do PSO
void aplica_textura_pso()
{
    // -- imagem original com area da superficie texturizada
    Mat im_result = im_source.clone();

    // -- transforma imagens das areas selecionadas em grayscale
    Mat selarea, transfm;
    cvtColor(im_selarea,selarea,CV_BGR2GRAY);
    cvtColor(im_transfm,transfm,CV_BGR2GRAY);

    // -- sobrescreve imagem original na area detectada
    for (int k=0; k<3; k++)
    {
        for (int i=0; i<im_source.rows; i++)
        {
            for (int j=0; j<im_source.cols; j++)
            {
                // se posicao faz parte da area detectada
                if ((selarea.at<uchar>(i,j) == MAX_BYTE) && (transfm.at<uchar>(i,j) > 10))
                    im_result.at<Vec3b>(i,j)[k] = im_transfm.at<Vec3b>(i,j)[k];
            }
        }
    }

    // -- mostra imagem da superficie texturizada
    imshow(win_final_pso, im_result);

    // -- define globais
    im_trsfpso = im_result;
    im_trsftxr = im_result;
    im_work = im_result;
}

/// calcula qualidade baseado na area do fecho e outra fornecida
float calc_qualid_area(Mat res)
{
    float qualidade=0;
    Mat shg;
    cvtColor(im_selarea,shg,CV_BGR2GRAY);
    Mat sr = shg - res;
    qualidade+=countNonZero(sr) * RULE_MED;
    Mat rs = res - shg;
    qualidade+=countNonZero(rs) * RULE_LOW;
    return qualidade;
}

/// calcula qualidade de uma solucao do PSO
float funcao_objetivo(vector<float> &solucao)
{

    // -- transforma solucao em coordenadas
    vector<Point> p(4);
    p[0].x = (int)solucao[0];
    p[0].y = (int)solucao[1];
    p[1].x = (int)solucao[2];
    p[1].y = (int)solucao[3];
    p[2].x = (int)solucao[4];
    p[2].y = (int)solucao[5];
    p[3].x = (int)solucao[6];
    p[3].y = (int)solucao[7];

    /* ----------------------------------------
     * PENALIDADES
     *  coord negativa / fora do plano - grave
     *  coord dentro de fecho convexo  - leve
     *  mapeamento corta fecho convexo - media
     * ---------------------------------------- */

    // -- inicializa variavel de penalizacao
    float pena=0;

    // -- coordenada negativa ou maior que limite dominio
    int lx=im_work.cols-1, ly=im_work.rows-1;
    if ((p[0].x < 0)  || (p[0].y < 0)) pena+=RULE_HIGH;
    if ((p[1].x < 0)  || (p[1].y < 0)) pena+=RULE_HIGH;
    if ((p[2].x < 0)  || (p[2].y < 0)) pena+=RULE_HIGH;
    if ((p[3].x < 0)  || (p[3].y < 0)) pena+=RULE_HIGH;
    if ((p[0].x > lx) || (p[0].y > ly)) pena+=RULE_HIGH;
    if ((p[1].x > lx) || (p[1].y > ly)) pena+=RULE_HIGH;
    if ((p[2].x > lx) || (p[2].y > ly)) pena+=RULE_HIGH;
    if ((p[3].x > lx) || (p[3].y > ly)) pena+=RULE_HIGH;

    // -- penalidade grave retorno erro maximo
    if (pena > 0) return MAX_ERRO;

    // -- verifica cobertura em relacao a area de fecho convexo
    // -- verifica proximidade com area do fecho convexo
    pena += calc_qualid_area(cria_poligono(p));

    // -- penalidade final
    return pena;
}

/// executa algoritmo do PSO para arquitetar pontos
void executa_pso()
{

    // -- parametros fixos do PSO
    PsoConfig config;
    config.tamanhoParticula = 8;
    config.dominioMin = 0;
    config.dominioMaxX = im_work.cols-1;
    config.dominioMaxY = im_work.rows-1;
    config.coeficienteInercia = 1;
    config.influenciaParticula = 0.8;
    config.influenciaVizinhanca = 0.8;
    config.funcaoObjetivo = funcao_objetivo;

    // -- parametros ajustaveis do PSO
    config.maxIteracoes = 50;
    config.tamanhoEnxame = 100;
    config.velocidadeMaxima = 10;
    config.minQualidade = QLD_ACEIT;
    config.maxCongelamento = 45;

    // -- executa algoritmo PSO
    printf("Iniciando processo PSO para mapeamento dos pontos...\n");
    Pso pso;
    vector<float> solucao = pso.executa(config);

    // -- melhor resultado
    vector<Point> p(4);
    p[0].x = (int)solucao[0];
    p[0].y = (int)solucao[1];
    p[1].x = (int)solucao[2];
    p[1].y = (int)solucao[3];
    p[2].x = (int)solucao[4];
    p[2].y = (int)solucao[5];
    p[3].x = (int)solucao[6];
    p[3].y = (int)solucao[7];

    // -- ordena pontos em sentido anti-horario
    ptsDest = ordena_pontos_ccw(p);
    printf("Coordenadas lBest:\n");
    for(int i=0;i<4;i++)
    {
        printf("[%d] %d %d\n",i,ptsDest[i].x,ptsDest[i].y);
    }

    // -- cria imagem com area detectada
    im_selrpso = cria_poligono(ptsDest);
    if(pdi_trace) imshow(win_area_pso,im_selrpso);

}


/*
 * ----------------------------------------------
 * MORFOLOGIA
 * ----------------------------------------------
 */

/// operacao logica AND entre matrizes
Mat oper_and(Mat ent1, Mat ent2)
{

    // declaracoes
    Mat saida(ent1.rows, ent1.cols, CV_8U, Scalar(0));

    // varre matriz imagem
    for(int i = 0; i < ent1.rows; i++)
    {
        for(int j = 0; j < ent1.cols; j++)
        {
            if ((ent1.at<uchar>(i,j) == MAX_BYTE) && (ent2.at<uchar>(i,j) == MAX_BYTE))
            {
                saida.at<uchar>(i,j) = MAX_BYTE;
            }
        }
    }
    return saida;
}

/// operacao logica OR entre matrizes
Mat oper_or(Mat ent1, Mat ent2)
{

    // declaracoes
    Mat saida(ent1.rows, ent1.cols, CV_8U, Scalar(0));

    // varre matriz imagem
    for(int i = 0; i < ent1.rows; i++)
    {
        for(int j = 0; j < ent1.cols; j++)
        {
            if ((ent1.at<uchar>(i,j) == MAX_BYTE) || (ent2.at<uchar>(i,j) == MAX_BYTE))
            {
                saida.at<uchar>(i,j) = MAX_BYTE;
            }
        }
    }
    return saida;
}

/// aplica erosao - imagem parametrizada
Mat morf_erosion(Mat im_area, int es=3)
{
    Mat erosion_dst;

    // define mascara
    int ts = 2*es-1;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( ts, ts ),
                                       Point( es, es ) );

    // aplica erosao
    erode(im_area, erosion_dst, element);
    return erosion_dst;
}

/// aplica erosao - imagem de trabalho
void morf_erosion()
{
    // define mascara
    int thresh_erosion = thresh_dilation-1;
    int ts = 2*thresh_erosion+1;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( ts, ts ),
                                       Point( thresh_erosion, thresh_erosion ) );

    // aplica erosao
    erode(im_work, im_work, element);
}

/// aplica dilatacao - imagem de parametrizada
Mat morf_dilation(Mat im_area, int es=3)
{
    Mat dilate_dst;

    // define mascara
    int ts = 2*es-1;
    Mat element = getStructuringElement( MORPH_RECT,
                                       Size( ts, ts ),
                                       Point( es, es ) );
    // aplica dilatacao
    dilate( im_area, dilate_dst, element );
    return dilate_dst;
}

/// aplica dilatacao - imagem de trabalho
void morf_dilation()
{
    // define mascara
    int ts = 2*thresh_dilation+1;
    Mat element = getStructuringElement( MORPH_RECT,
                                       Size( ts, ts ),
                                       Point( thresh_dilation, thresh_dilation ) );
    // aplica dilatacao
    dilate( im_work, im_work, element );
}

/// aplica morfologia de abertura
void morf_abertura()
{
    morf_erosion();
    morf_dilation();
}

/// aplica morfologia de fechamento
void morf_fechamento()
{
    morf_dilation();
    morf_erosion();
}

/// extrai fronteira de img binarizada
Mat morf_fronteira(Mat im_area)
{
    Mat area = morf_dilation(im_area);
    Mat front = morf_erosion(area,3);
    Mat im_subt = area - front;
    cvtColor(im_subt,im_subt,CV_BGR2GRAY);
    return im_subt;
}


/*
 * ----------------------------------------------
 * OPERACOES PRINCIPAIS
 * ----------------------------------------------
 */

/// calcula distancia euclidiano entre 2 pontos
int calcula_dist_eucl(Point2f a, Point2f b)
{
    return (int)(norm(a-b));
}

/// converte imagem para HSV e ajusta seus valores
void controleHSV()
{
    Mat im_hsv = Mat::zeros(src_size,CV_8UC3);
    cvtColor(im_work, im_hsv, CV_BGR2HSV);
    for (int i=0; i < im_hsv.rows ; i++)
    {
        for(int j=0; j < im_hsv.cols; j++)
        {
            for(int k=0; k < 3; k++)
            {
                int imgk = im_hsv.at<Vec3b>(i,j)[k];
                imgk += thresh_hsv[k]-255;
                if (imgk < 0) imgk = 0;
                else if (imgk > 255) imgk = 255;
                im_hsv.at<Vec3b>(i,j)[k] = imgk;
            }
        }
    }
    cvtColor(im_hsv, im_hsv, CV_HSV2BGR);

    // mostra imagem alterada
    if(pdi_trace) imshow(win_tbar_hsv, im_hsv);

    // seta global
    im_work = im_hsv;
}

/// converte imagem para escalas de cinza
void converte_cinza()
{
    Mat im_gray(src_size, CV_8U);

    // converte para grayscale
    cvtColor(im_work, im_gray, CV_BGR2GRAY);
    if(pdi_trace) imshow(win_tbar_ajuste, im_gray);

    // seta global
    im_work = im_gray;
}

/// visualiza histograma (grafico)
void plot_histograma(const char* win_name)
{
    int histSize = 256;
    float range[] = { 0, 255 };
    const float *ranges[] = { range };

    // -- calcula histograma
    Mat gray=im_work.clone();
    MatND hist;
    calcHist(&gray, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false);

    // -- plota histograma
    int hist_w = 256; int hist_h = 100;
    int bin_w = cvRound( (double) hist_w/histSize );
    Mat histImage(hist_h, hist_w, CV_8UC1, Scalar(0));
    normalize(hist,hist,0,histImage.rows,NORM_MINMAX, -1, Mat());
    for( int i = 1; i < histSize; i++ )
    {
        line(histImage,
             Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
             Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
             Scalar(255));
    }

    // -- mostra resultado
    if(pdi_trace) imshow(win_name, histImage);
}

/// aplica equalizacao de histograma no pro-processamento
void equaliza_histograma_pre()
{
    // aplica histograma
    Mat im_hist;
    if (thresh_histog)
        equalizeHist(im_work, im_hist);
    else im_hist = im_work;

    // mostra imagem e seta principal
    if(pdi_trace) imshow(win_tbar_ajuste, im_hist);
    im_work = im_hist;
}

/// aplica equalizacao de histograma na img final
void equaliza_histograma_pos()
{
    Mat im_hist;
    if (thresh_hiscor)
    {
        vector<Mat> channels;
        split(im_work ,channels);
        for (int i=0; i<3; i++)
            equalizeHist(channels[i], channels[i]);
        merge(channels, im_hist);
    }
    else im_hist = im_work.clone();

    // -- mostra imagem e seta principal
    if(pdi_trace) imshow(win_main_final, im_hist);
    im_work = im_hist;
}

/// balanceia iluminacao na imagem com a textura aplicada
void equaliza_ilum_textr()
{
    // -- converte blobs para grayscale
    // -- converte image final para HSV
    Mat sela, tran, src, wrk, srcg, wrkg;
    cvtColor(im_selarea, sela, CV_BGR2GRAY);
    cvtColor(im_transfm, tran, CV_BGR2GRAY);
    cvtColor(im_source, src, CV_BGR2HSV);
    cvtColor(im_work, wrk, CV_BGR2HSV);
    cvtColor(im_source, srcg, CV_BGR2GRAY);
    cvtColor(im_work, wrkg, CV_BGR2GRAY);
    Mat out = wrk.clone();
    for (int i=0; i < out.rows ; i++)
    {
        for(int j=0; j < out.cols; j++)
        {
            if (sela.at<uchar>(i,j)==255)
            {
                //if (!mp_transfd || (mp_transfd && (tran.at<uchar>(i,j) > 0)) )
                if (tran.at<uchar>(i,j) > 0)
                {
                    // -- adequa valor de região texturizada de acordo com orig.
                    int s = src.at<Vec3b>(i,j)[2];
                    int d = wrk.at<Vec3b>(i,j)[2];
                    //int p = 1.5*s-(255-d);
                    int p = (thresh_ilumn/10.0f)*s-(255-d);
                    if (p < 0) p = 0;
                    else if (p > 255) p = 255;
                    out.at<Vec3b>(i,j)[2] = p;
                }
            }
        }
    }
    cvtColor(out, out, CV_HSV2BGR);
    im_work = out.clone();

    if(pdi_trace) imshow("Textura",im_wallpaper);

    namedWindow(win_main_final);
    createTrackbar("Iluminacao", win_main_final, &thresh_ilumn, 100, executa);
}

/// suaviza bordas da região texturizada para refinamento
void suaviza_bordas_textr()
{
    // -- extrai contorno da area da aplicacao
    Mat front = morf_fronteira(im_selarea);
    Mat result;
    medianBlur(im_work, result, 3);
    blur(result, result, Size(3,3));
    for(int i=0; i<im_work.rows; i++)
    {
        for (int j=0; j<im_work.cols; j++)
        {
            if (front.at<uchar>(i,j) == MAX_BYTE)
            {
                im_work.at<Vec3b>(i,j) = result.at<Vec3b>(i,j);
            }
        }
    }
}

/// ajusta brilho/contraste da imagem
void ajusta_brilho_contraste()
{
    Mat im_sat(src_size, CV_8U, Scalar(0,0,0));

    // -- altera contraste | brilho
    for( int y = 0; y < im_work.rows; y++ )
    {
        for( int x = 0; x < im_work.cols; x++ )
        {
            double sat_value = ((double)thresh_contrs * 0.1) * (im_work.at<uchar>(y,x) ) + thresh_brilho;
            im_sat.at<uchar>(y,x) = saturate_cast<uchar>(sat_value);
        }
    }

    // -- mostra imagem alterada e seta global
    if(pdi_trace) imshow(win_tbar_ajuste, im_sat);
    im_work = im_sat;
}

/// ajusta gamma da imagem
void correcao_gamma()
{
    Mat im_gamma;
    // -- verifica se imagem eh uchar
    CV_Assert(im_work.data);
    CV_Assert(im_work.depth() != sizeof(uchar));

    // cria tabela de pesquisa
    unsigned char lut[256];
    float fGamma = thresh_gamma*1.0f/10;
    for (int i = 0; i < 256; i++)
        lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);

    im_gamma = im_work.clone();
    const int channels = im_gamma.channels();
    switch (channels)
    {
        case 1:
        {
            MatIterator_<uchar> it, end;
            for (it = im_gamma.begin<uchar>(), end = im_gamma.end<uchar>(); it != end; it++)
                *it = lut[(*it)];
            break;
        }
        case 3:
        {
            MatIterator_<Vec3b> it, end;
            for (it = im_gamma.begin<Vec3b>(), end = im_gamma.end<Vec3b>(); it != end; it++)
            {
                (*it)[0] = lut[((*it)[0])];
                (*it)[1] = lut[((*it)[1])];
                (*it)[2] = lut[((*it)[2])];
            }
            break;
        }
    }

    // -- mostra imagem e define global
    if(pdi_trace) imshow(win_tbar_ajuste, im_gamma);
    im_work = im_gamma;
}

/// aplica sharpening na imagem
void ajusta_nitidez()
{

    Mat im_sharp = im_work.clone();

    // -- verifica se funcao esta ativada
    if (thresh_sharp == 1)
    {

        // -- filtro laplaciano - aproximacao da 2a derivada
        Mat kernel = (Mat_<float>(3,3) <<
                1,  1, 1,
                1, -8, 1,
                1,  1, 1);
        Mat im_laplacian;
        Mat sharp = im_work.clone();
        filter2D(sharp, im_laplacian, CV_32F, kernel);
        im_work.convertTo(sharp, CV_32F);
        im_sharp = sharp - im_laplacian;

        // -- converte de volta para 8bits
        im_sharp.convertTo(im_sharp, CV_8UC3);
    }

    // -- mostra imagem e define global
    if(pdi_trace) imshow(win_tbar_ajuste, im_sharp);
    im_work = im_sharp;
}

/// aplica blur mediano
void aplica_median_blur()
{
    // -- aplica blur
    if (thresh_blur > 0)
    {
        int ksize = thresh_blur*2+1;
        Mat im_mblur;
        medianBlur(im_work, im_mblur, ksize);
        im_work = im_mblur;
        if(pdi_trace) imshow(win_tbar_ajuste, im_work);
    }
}

/// aplica filtros passabaixa
void aplica_filtros()
{
    // -- aplica blur
    Mat im_blur, im_gblur, im_mblur, im_bblur;
    if (thresh_blur == 0)
    {
        im_blur=im_gblur=im_mblur=im_bblur=im_work.clone();
    } else {
        int ksize = thresh_blur*2+1;
        GaussianBlur(im_work, im_gblur, Size(ksize, ksize), 100, 100);
        bilateralFilter(im_work, im_bblur, ksize, 100, 10);
        medianBlur(im_work, im_mblur, ksize);
        blur(im_work, im_blur, Size(ksize, ksize));
    }

    // -- mostra resultado
    if(pdi_trace)
    {
        imshow(win_tbar_ajuste, im_mblur);
        imshow("Gaussiano", im_gblur);
        imshow("Mediano", im_mblur);
        imshow("Bilateral", im_bblur);
        imshow("Homogeneo", im_blur);
    }

    // -- seta principal - sempre median blur
    im_work = im_mblur;
}

/*
 * ----------------------------------------------
 * OPERACOES EM IMAGENS BINARIAS
 * ----------------------------------------------
 */

/// converte a imagem para binaria
void binariza_imagem()
{
    // aplica binarizacao
    Mat im_binaria;
    threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_BINARY);

    // mostra imagem e seta principal
    if(pdi_trace) imshow(win_tbar_proces, im_binaria);
    im_work = im_binaria;
}

/// converte a imagem utilizando binarizacao adaptiva
void binar_adaptiva_auto()
{
    Mat im_binaria;
    int blk = (thresh_binblk > 1) ? (thresh_binblk * 2 - 1) : 3;
    adaptiveThreshold(im_work, im_binaria, MAX_BYTE, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, blk, thresh_binc);
    if(pdi_trace) imshow(win_tbar_binar, im_binaria);
    im_work=im_binaria;
}

/// aplica binarizacao de acordo com opcao
void binarizacao_adaptiva()
{
    Mat im_binaria;

    int blk = (thresh_binblk > 1) ? (thresh_binblk * 2 - 1) : 3;
    switch(thresh_binar)
    {
        case 0:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_BINARY_INV);
            break;
        case 1:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_BINARY_INV | THRESH_OTSU);
            break;
        case 2:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_TOZERO_INV);
            break;
        case 3:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_TOZERO_INV | THRESH_OTSU);
            break;
        case 4:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_TRUNC);
            break;
        case 5:
            threshold(im_work, im_binaria, 1, MAX_BYTE, THRESH_TRUNC | THRESH_OTSU);
            break;
        case 6:
            adaptiveThreshold(im_work, im_binaria, MAX_BYTE, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, blk, thresh_binc);
            break;
        case 7:
            adaptiveThreshold(im_work, im_binaria, MAX_BYTE, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, blk, thresh_binc);
            break;
    }
    if(pdi_trace) imshow(win_tbar_binar, im_binaria);
}

/// inverte bits de imagem binaria
void inverte_imagem()
{
    Mat im_inverte(im_work.rows, im_work.cols, CV_8UC1);
    for (int i=0; i < im_work.rows; i++)
    {
        for (int j=0; j < im_work.cols; j++)
        {
            im_inverte.at<uchar>(i, j) =
                (im_work.at<uchar>(i, j) == MAX_BYTE) ? 0 : MAX_BYTE;
        }
    }

    // -- mostra imagem e seta principal
    if(pdi_trace) imshow(win_tbar_proces, im_inverte);
    im_work = im_inverte;
}

/// aplica operador canny para extracao de contornos
void aplica_canny()
{
    Mat im_canny(src_size, CV_8U);

    // -- detecta bordas utilizando Canny
    Canny( im_work, im_canny, thresh_canny1, thresh_canny2, 3 );

    // -- mostra imagem e seta principal
    if(pdi_trace) imshow(win_tbar_proces, im_canny);
    im_work = im_canny;


    // -- hough lines
    Mat im_hlines;
    cvtColor(im_canny, im_hlines, CV_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(im_canny, lines, 1, CV_PI/180, 50, 50, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( im_hlines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    if(pdi_trace) imshow("Hough Lines", im_hlines);
}

/// desenha linha nas extremidades da imagem binarizada
void desenha_margem()
{
    Mat im_margem = im_work;
    for (int i=0; i < im_work.rows; i++)
    {
        im_margem.at<uchar>(i, 0) = MAX_BYTE;
        im_margem.at<uchar>(i, (im_work.cols-1)) = MAX_BYTE;
    }
    for (int i=0; i < im_work.cols; i++)
    {
        im_margem.at<uchar>(0, i) = MAX_BYTE;
        im_margem.at<uchar>((im_work.rows-1), i) = MAX_BYTE;
    }

    // -- mostra imagem e seta principal
    if(pdi_trace) imshow(win_tbar_proces, im_margem);
    im_work = im_margem;
}

/// aplica morfologia de acordo com opcao
void aplica_morfologias()
{

    // -- 0F 1A 2FF 3AA 4FA 5AF 6FE 7FD 8AE 9AD
    // 0 - fechamento
    // 1 - abertura
    // 2 - fechamento/fechamento
    // 3 - abertura/abertura
    // 4 - fechamento/abertura
    // 5 - abertura/fechamento
    // 6 - fechamento/erosao
    // 7 - fechamento/dilatacao
    // 8 - abertura/erosao
    // 9 - abertura/dilatacao
    switch(thresh_abfe)
    {
        case 0:
            morf_fechamento();
            break;
        case 1:
            morf_abertura();
            break;
        case 2:
            morf_fechamento();
            morf_fechamento();
            break;
        case 3:
            morf_abertura();
            morf_abertura();
            break;
        case 4:
            morf_fechamento();
            morf_abertura();
            break;
        case 5:
            morf_abertura();
            morf_fechamento();
            break;
        case 6:
            morf_fechamento();
            morf_erosion();
            break;
        case 7:
            morf_fechamento();
            morf_dilation();
            break;
        case 8:
            morf_abertura();
            morf_erosion();
            break;
        case 9:
            morf_abertura();
            morf_dilation();
            break;
    }

    // -- mostra imagem
    if(pdi_trace) imshow(win_tbar_morfo, im_work);
}

/// busca contornos na imagem binarizada e extrai as respectivas
/// bounding box e fechos convexos de cada forma
void busca_contornos()
{
    RNG rng(12345);

    // -- rastreia contornos
    vector<vector<Point> > contours0, contours1;
    findContours(im_work, contours0, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // -- guarda contornos com areas de tamanho valido
    for( size_t k = 0; k < contours0.size(); k++ )
    {
        if ( contourArea(contours0[k]) > MIN_CONTOUR_SIZE )
            contours1.push_back(contours0[k]);
    }
    contours.resize(contours1.size());

    // -- bounding box
    vector<Rect> boundRect(contours.size());

    // -- convex hull
    vector<vector<Point> > hull(contours.size());

    // -- seta vetores de cada categoria
    for(size_t k = 0; k < contours1.size(); k++)
    {
        approxPolyDP(Mat(contours1[k]), contours[k], thresh_apxpoly, 0);
        boundRect[k] = boundingRect(Mat(contours[k]));
        convexHull(Mat(contours[k]), hull[k], false);
    }

    // -- rotula contornos (cores) e desenha
    Mat im_contour = Mat::zeros(src_size, CV_8UC3 );
    Mat im_cvxhull = Mat::zeros(src_size, CV_8UC3 );
    Mat im_boundbx = Mat::zeros(src_size, CV_8UC3 );
    for( int i = 0; i< (int)contours.size(); i++ )
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        drawContours(im_contour, contours, i, color, CV_FILLED, 8, hierarchy, 0, Point());
        drawContours(im_cvxhull, hull, i, color, 2, 8, vector<Vec4i>(), 0, Point());
        rectangle(im_boundbx, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }

    // -- mostra imagens
    if(pdi_trace) imshow(win_tbar_contor, im_contour);
    if(pdi_trace) imshow(win_tbar_cvhull, im_cvxhull);
    if(pdi_trace) imshow(win_tbar_bdnbox, im_boundbx);

    // -- seta globais
    im_work = im_contour;
    bRect = boundRect;
    cHull = hull;
    im_hull = im_cvxhull;
}


/*
 * ----------------------------------------------
 * TEXTURA
 * ----------------------------------------------
 */

/// valida area selecionada pelo usuario e guarda superficie
void valida_area_aplic()
{
    selected=0;

    // -- seleciona menor superficie valida na area do clique
    for (int i=contours.size()-1; ((i>=0) && (!selected)); i--)
    {
        if (pointPolygonTest(contours[i],mouse_click,0) >= 0)
        {
            selected=i;
        }
    }

    // -- desenha area do bounding box da superficie
    im_selbbox = Mat::zeros(src_size, CV_8UC3);
    rectangle(im_selbbox, bRect[selected].tl(), bRect[selected].br(), Scalar(255,255,255), CV_FILLED, 8, 0 );
    selRect = bRect[selected];

    // -- desenha area do contorno da superficie
    im_selarea = Mat::zeros(src_size, CV_8UC3);
    drawContours( im_selarea, contours, selected, Scalar(255,255,255), CV_FILLED, 8, hierarchy, 0, Point() );
    selCont = contours[selected];

    // -- subtrai contornos filho do contorno selecionado
    for(int i=selected+1; i<contours.size(); i++)
    {
        drawContours( im_selarea, contours, i, Scalar(0,0,0), CV_FILLED, 8, hierarchy, 0, Point() );
    }

    // -- desenha area do contorno fechado da superficie
    im_selhull = Mat::zeros(src_size, CV_8UC3);
    approxPolyDP(Mat(cHull[selected]), cHull[selected], 15, 0);
    drawContours( im_selhull, cHull, selected, Scalar(255,255,255), CV_FILLED, 8, vector<Vec4i>(), 0, Point() );
    selHull = cHull[selected];

    // -- mostra areas pintadas dos objetos
    if(pdi_trace) imshow(win_area_bbox, im_selbbox);
    if(pdi_trace) imshow(win_area_cont, im_selarea);
    if(pdi_trace) imshow(win_area_hull, im_selhull);
}

/// aplica textura na area equivalente a bounding box da superficie
void texturiza_area_bbox()
{
    Mat im_result = Mat::zeros(src_size, CV_8UC3);

    // -- preenche imagem da superficie com a textura (lado-a-lado)
    Mat selbbox;
    cvtColor(im_selbbox,selbbox,CV_BGR2GRAY);
    for (int k=0; k<3; k++)
    {
        for (int i=0; i<im_work.rows; i++)
        {
            for (int j=0; j<im_work.cols; j++)
            {
                if (selbbox.at<uchar>(i,j) > 0)
                {
                    //im_result.at<Vec3b>(i,j)[k] = MAX_BYTE;
                    im_result.at<Vec3b>(i,j)[k] = im_wallpaper.at<Vec3b>(i,j)[k];
                }
            }
        }
    }

    // -- mostra imagem da superficie texturizada
    if(pdi_trace) imshow(win_text_bbox, im_result);

    // -- transforma area com textura
    im_areatxt = im_result.clone();
}

/// mapeia pontos origem de uma transformacao
void define_pontos_orig()
{
    ptsOrig.resize(4);
    ptsOrig[0] = Point2f(selRect.x, selRect.y);
    ptsOrig[1] = Point2f(selRect.x, selRect.y+selRect.height);
    ptsOrig[2] = Point2f(selRect.x+selRect.width, selRect.y+selRect.height);
    ptsOrig[3] = Point2f(selRect.x+selRect.width, selRect.y);
    for(int i=0;i<4;i++)
    {
        circle(im_selbbox, ptsOrig[i], 5,  Scalar(255,0,0), 2, 8, 0);
        if(pdi_trace) imshow(win_area_bbox, im_selbbox);
    }
}

/// mapeia pontos destino da transformacao (metodo aprox. a fecho convexo)
void define_pontos_dest()
{
    // -- para cada ponto da imagem, calcula distancia dos
    // -- pontos das bbox e guarda mais proximo
    vector<int> h(4,INT_MAX), hh(4,INT_MAX);
    vector<Point2f> ph;
    ph.resize(4);
    for( int i = 0; i < selHull.size(); i++ )
    {
        for (int k=0; k<4; k++)
        {
            h[k] = calcula_dist_eucl(ptsOrig[k], selHull[i]);
            if (h[k] < hh[k])
            {
                ph[k] = selHull[i];
                hh[k] = h[k];
            }
        }
    }
    // -- define pontos destino
    ptsDest.resize(4);
    /*for (int k=0; k<4; k++)
    {
        ptsDest[k]=ph[k];
    }*/
    // -- acrescenta margem
    int mg=20;
    int r=im_source.rows-1, c=im_source.cols-1;
    ptsDest[0]=Point( ph[0].x-mg , ph[0].y-mg );
    ptsDest[1]=Point( ph[1].x-mg , ph[1].y+mg );
    ptsDest[2]=Point( ph[2].x+mg , ph[2].y+mg );
    ptsDest[3]=Point( ph[3].x+mg , ph[3].y-mg );

    if (ptsDest[0].x < 0) ptsDest[0].x = 0;
    if (ptsDest[0].y < 0) ptsDest[0].y = 0;
    if (ptsDest[1].x < 0) ptsDest[1].x = 0;
    if (ptsDest[1].y > r) ptsDest[1].y = r;
    if (ptsDest[2].x > c) ptsDest[2].x = c;
    if (ptsDest[2].y > r) ptsDest[2].y = r;
    if (ptsDest[3].x > c) ptsDest[3].x = c;
    if (ptsDest[3].y < 0) ptsDest[3].y = 0;

    // -- desenha circulos nos pontos mapeados na imagem do fecho convexo
    for( int k=0; k<4; k++ )
    {
        circle(im_selhull, Point(ph[k].x, ph[k].y), 5, Scalar(0,0,(k+1)*60), 2, 8, 0);
    }

    // -- cria imagem com area transformada
    im_seltran = cria_figura(src_size,ptsDest);

    // -- mostra imagem do fecho com as posicoes destino
    if(pdi_trace) imshow(win_area_tran, im_seltran);
    if(pdi_trace) imshow(win_area_hull, im_selhull);
}

/// replica textura em imagem com dimensoes da foto (area total)
void cria_fundo_textura()
{
    // -- imagem com tamanho da foto, preenchida com textura lado-a-lado
    Mat im_txtotal(src_size, CV_8UC3, Scalar(0,0,0));

    // -- preenche imagem com textura lado-a-lado
    for (int k=0; k<3; k++)
    {
        int m=0;
        for (int i=0; i<im_txtotal.rows; i++)
        {
            int n=0;
            for (int j=0; j<im_txtotal.cols; j++)
            {
                im_txtotal.at<Vec3b>(i,j)[k] = im_texture.at<Vec3b>(m,n)[k];
                n = (n < im_texture.cols-1) ? n+1 : 0;
            }
            m = (m < im_texture.rows-2) ? m+1 : 0;
        }
    }

    // -- mostra imagem
    if(pdi_trace) imshow("TxTotal", im_txtotal);

    // -- define estado inicial da imagem de transformacao
    im_transfm = im_txtotal.clone();

    // -- define global
    im_wallpaper = im_txtotal.clone();
}

/// aplica textura com e sem transformacao, sobre a imagem original
/// na regiao selecionada pelo usuario
void aplica_textura()
{
    // -- imagem original com area da superficie texturizada
    Mat im_resultt = im_source.clone();

    // -- transforma imagens das areas selecionadas em grayscale
    Mat selarea, transfm;
    cvtColor(im_selarea,selarea,CV_BGR2GRAY);
    cvtColor(im_transfm,transfm,CV_BGR2GRAY);

    // -- sobrescreve imagem original na area detectada
    for (int k=0; k<3; k++)
    {
        for (int i=0; i<im_source.rows; i++)
        {
            for (int j=0; j<im_source.cols; j++)
            {
                // -- se posicao faz parte da area detectada
                if ((selarea.at<uchar>(i,j) == MAX_BYTE) && (transfm.at<uchar>(i,j) > 0))
                {
                    im_resultt.at<Vec3b>(i,j)[k] = im_transfm.at<Vec3b>(i,j)[k];
                }
            }
        }
    }

    // -- mostra imagem da superficie texturizada
    imshow(win_final_transf, im_resultt);

    // -- define globais
    im_trsfhul = im_resultt;
    im_trsftxr = im_resultt;
    im_work = im_resultt;
}


/*
 * ----------------------------------------------
 * MAPEAMENTO/TRANSFORMACAO ESPACIAL
 * ----------------------------------------------
 */

/// aplica transformacao perspectiva, a partir de pontos mapeados
void transformacao_espacial()
{
    for(int k=0; k<4; k++)
    {
        inputQuad[k] = ptsOrig[k];
        outputQuad[k] = ptsDest[k];
    }
    Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(im_areatxt, im_transfm, lambda, src_size);
    if (pdi_trace) imshow(win_tran_auto, im_transfm);
}

/// mostra resultado ou volta para estado de transformacao
void mostra_resultado(int e, int x, int y, int, void*)
{

    // -- botao esquerdo: decide como final
    if (e == EVENT_LBUTTONDOWN )
    {
        printf("Mostra imagem resultante...\n\n", x, y);
        seta_fluxo(EF_RESULT);
    }
    // -- volta para estado de texturizacao
    else if (e == EVENT_RBUTTONDOWN )
    {
        printf("Volta para transformacao da textura...\n\n", x, y);
        seta_fluxo(EF_TEXTURA);
    }
}

/// estima qualidade das transformacoes disponiveis e escolhe a melhor
void calc_melhor_transf()
{
    // -- qualidade pso
    float qualid_pso = calc_qualid_area(im_selrpso);

    // -- qualidade hull
    float qualid_ft = calc_qualid_area(im_seltran);

    // -- seleciona melhor qualidade
    float qbest=MAX_ERRO;
    int qs=0;
    if (qualid_pso < qbest)
    {
        qbest=qualid_pso;
        qs=Q_PSO;
    }
    if (qualid_ft < qbest)
    {
        qbest=qualid_ft;
        qs=Q_HULL;
    }

    // -- imprime melhor qualidade
    printf("Qualidade de cada transformacao:\n");
    printf("PSO\t%.f\n",qualid_pso);
    printf("C.Hull\t%.f\n",qualid_ft);
    printf("Melhor transformacao escolhida: ");
    switch(qs)
    {
        case Q_PSO:
            printf("PSO\n");
            im_work = im_trsfpso;
            im_selfina = im_selrpso;
            break;
        case Q_HULL:
            printf("C.Hull\n");
            im_work = im_trsfhul;
            im_selfina = im_selhull;
            break;
    }
}


/*
 * ----------------------------------------------
 * FUNCOES DE CALLBACK
 * ----------------------------------------------
 */

/// aplica transformacao perspectiva manualmente
void callback_transform(int e, int x, int y, int, void*)
{

    // -- botao esquerdo: indica ponto para mapeamento
    if (e == EVENT_LBUTTONDOWN )
    {
        printf("[transform.manual] Click %d detectado (%d, %d)\n", warp_st, x, y);
        if (flux_st == EF_TEXTURA)
            destroyAllWindows();

        inputQuad[warp_st] = ptsOrig[warp_st];
        outputQuad[warp_st] = Point2f(x,y);

        // -- se ja indicou os 4 pontos, transforma
        if (warp_st == 3)
        {
            printf("Exibe resultado...\n\n");

            // -- aplica transformacao
            Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);
            warpPerspective(im_areatxt, im_transfm, lambda, src_size);

            // -- define blob final
            im_seltman = cria_poligono(pf2vp(outputQuad,4));
            im_selfina = im_seltman;

            // -- mostra imagem
            if (pdi_trace) imshow(win_tran_manual, im_transfm);

            // -- reseta booleanos
            warp_st=0;
            mp_transfd=true;
        }
        // -- senao, continua lendo
        else {
            flux_st = EF_MANUAL;
            warp_st++;
            mp_transfd=0;
        }
        executa(0,0);
    }
    // -- botao do meio: considera transformacao autom. como final
    else if (e == EVENT_MBUTTONDOWN )
    {
        //printf("[transfm] click detectado: %d, %d\n", x, y);
        seta_fluxo(EF_RESULT);
    }
    // -- botao direito: executa PSO para novo mapeamento
    else if (e == EVENT_RBUTTONDOWN )
    {
        seta_fluxo(EF_PSO);
    }
}

/// captura eventos (click do usuario) para tomadas de decisao
void callback_executa(int event, int x, int y, int flags, void* userdata)
{

    // -- botao esquerdo (clica): seleciona regiao
    if (event == EVENT_LBUTTONDOWN)
    {
        mouse_click.x = x;
        mouse_click.y = y;
        printf("Detectado click (ini): x=%d, y=%d\n", mouse_click.x, mouse_click.y);
    }

    // -- botao esquerdo (solta): seleciona regiao
    else if (event == EVENT_LBUTTONUP)
    {
        mouse_rls.x = x;
        mouse_rls.y = y;
        printf("Detectado click (fim): x=%d, y=%d\n", mouse_rls.x, mouse_rls.y);
        if (flux_st != EF_TEXTURA)
            seta_fluxo(EF_TEXTURA);
        else executa(0,0);
    }

    // -- botao direito: reseta fluxo
    else if (event == EVENT_RBUTTONDOWN)
    {
        mouse_click.x = 0;
        mouse_click.y = 0;
        mouse_st=0;
        mp_transfd=0;
        im_selarea = Mat::zeros(src_size, CV_8UC3);
        printf("Reseta imagem/transformação.\n");
        seta_fluxo(EF_AJUSTE);
    }

    // -- botao central: calcula e mostra estimativas
    else if (event == EVENT_MBUTTONDOWN)
    {
        display_info();
        //grava_controles();
    }
}



/*
 * ----------------------------------------------
 * FLUXO PRINCIPAL / EVENTOS
 * ----------------------------------------------
 */


/// mostra informacoes pontuais sobre a execucao
void seta_fluxo(int new_st)
{
    destroyAllWindows();
    flux_st=new_st;
    if ((new_st == EF_AJUSTE) && pdi_trace)
        cria_track_bars();
    executa(0,0);
}


/// mostra informacoes pontuais sobre a execucao
void display_info()
{
    // -- ativa modo debug!
    if (pdi_trace)
    {
        pdi_trace = 0;
        printf("\n -- MODO DEBUG DESATIVADO --\n\n");
    }
    else
    {
        pdi_trace = 1;
        printf("\n -- MODO DEBUG ATIVADO --\n\n");
    }

    // -- area total, media e quantidade de contornos detectados
    float tc_area=0, mc_area=0;
    int qt_contours = (int)contours.size();
    for( int i = 0; i < qt_contours; i++ )
    {
        tc_area += (float)contourArea(contours[i]);
    }
    if ((tc_area + qt_contours) > 0)
    {
        mc_area = tc_area / qt_contours;
    }
    printf("------------------------------------\n");
    printf("Qtde.contornos........: %d\n", qt_contours);
    printf("Soma area contornos...: %.2f\n", tc_area);
    printf("Media area contornos..: %.2f\n", mc_area);
}

/// grava log (nao utilizado)
void grava_controles(int,void*)
{
    log_file << "======================= " << endl;
}

/// inicializacao do sistema: le imagens, redimensiona, cria
/// "modelo" de textura, define fluxo inicial e cria trackbars (teste)
void inicializa(Mat im_amb, Mat im_tex)
{
    // -- redimensiona imagens de entrada e define globais
    im_source = reduz_resolucao(im_amb);
    im_texture = reduz_textura(im_tex);

    // -- cria trackbars
    if (pdi_trace) cria_track_bars();

    // -- cria imagem texturizada
    cria_fundo_textura();

    // -- seta estado inicial do fluxo
    flux_st = EF_AJUSTE;
}

/// ciclo de processamento do programa (fluxo principal)
void executa(int,void*)
{

    // -- definicoes iniciais
    im_work = im_source.clone();

    // -- mostra imagem original e inicializa captura do mouse
    imshow(win_main_source, im_source);
    setMouseCallback(win_main_source, callback_executa, NULL);

    // -- verifica estado do ciclo
    if(!flux_st) st_result=0;
    switch(flux_st)
    {
        // -- ajustes de imagem
        case EF_AJUSTE:

            printf("\n>> Selecao do usuario.\n\n");
            printf("Clique na região desejada da imagem para texturizacao.\n\n");

            // -- ajustes de cor e filtragem
            controleHSV();
            converte_cinza();
            plot_histograma("Levels");
            equaliza_histograma_pre();
            ajusta_brilho_contraste();
            correcao_gamma();
            ajusta_nitidez();
            aplica_median_blur();
            im_ajuste = im_work;
            plot_histograma("Levels-pos");

            // -- tratamentos imagem binaria
            //binar_adaptiva_auto();
            aplica_canny();
            desenha_margem();

            // -- tratamentos morfologia
            //aplica_morf_manual();
            aplica_morfologias();

            // -- rastreamento de caracteristicas
            busca_contornos();
            break;

        // -- trabalha aplicacao da textura
        case EF_TEXTURA:

            printf("\n>> Validacao da Transformacao.\n");
            printf("Clique na imagem exibida, de acordo com as opcoes abaixo:\n");
            printf("  - botao esquerdo..: realiza transformacao manual;\n");
            printf("  - botao do meio...: aceita transformacao atual;\n");
            printf("  - botao direito...: realiza transformacao inteligente (PSO);\n");
            printf("-> Para selecionar outra região, indique na imagem original.\n\n");

            namedWindow(win_final_transf, CV_WINDOW_AUTOSIZE);
            setMouseCallback(win_final_transf, callback_transform, NULL);

            // -- verifica area selecionada
            valida_area_aplic();
            texturiza_area_bbox();
            morf_fronteira(im_selhull);

            // -- define pontos mapeamento
            define_pontos_orig();
            define_pontos_dest();

            // -- aplica textura na area e transforma
            transformacao_espacial();

            aplica_textura();
            break;

        // -- trabalha na selecao da area da textura
        case EF_MANUAL:

            printf("\n>> Transformacao Manual.\n");
            printf("Indique os 4 cantos da imagem, em sentido anti-horario.\n\n");
            namedWindow(win_final_transf, CV_WINDOW_AUTOSIZE);
            setMouseCallback(win_final_transf, callback_transform, NULL);

            // -- aplica textura na area
            aplica_textura();

            // -- indica pontos selecionados manualmente
            for (int i=0;i<warp_st;i++)
                circle(im_trsfhul, outputQuad[i], 3, Scalar(0, 0, 255), FILLED, LINE_AA);
            imshow(win_final_transf, im_trsfhul);

            // -- mostra resultado
            if (mp_transfd) seta_fluxo(EF_RESULT);

            break;

        // -- trabalha com PSO para mapeamento
        case EF_PSO:

            printf("\n>> Transformacao PSO.\n\n");

            // -- executa algoritmo PSO
            executa_pso();

            // -- realiza transformacao
            transform_espacial_pso();
            aplica_textura_pso();

            // -- calcula melhor transformacao
            //calc_melhor_transf();

            // -- mostra resultado
            seta_fluxo(EF_RESULT);

            break;

        // -- mostra imagem resultante e controla iluminacao
        case EF_RESULT:

            if (!st_result)
            {
                printf("\n>> Exibindo resultado...\n");
                printf("Ajuste a iluminação da textura se desejar.\n");
                printf("-> Para retornar a etapa anterior, clique com ");
                printf("botao direito na imagem resultante.\n\n");
                st_result++;
            }
            namedWindow(win_main_final, CV_WINDOW_AUTOSIZE);
            setMouseCallback(win_main_final, mostra_resultado, NULL);

            // -- trabalha com imagem texturizada
            im_work = im_trsftxr;

            // -- refinamentos
            equaliza_ilum_textr();
            suaviza_bordas_textr();

            // -- mostra imagem final
            imshow(win_main_final, im_work);
            break;
    }
}

/* ##############################################
 *
 * FIM
 *
 * ##############################################
 */

#endif // FUNCTIONS_H
