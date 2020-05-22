// Primeiro terreno controlado por mouse e teclado
// derivado de https://github.com/SunilRao01/OSGFirstPersonController
// vpbmaster -v 0.005 -l 6 -d BRalt.tif -t br.png -o terreno1/BR.osgb
// Lista de classes: https://codedocs.xyz/openscenegraph/OpenSceneGraph/annotated.html

/* To-do:
 * 
 . Tecla Home: voltar pra posição inicial
 . Posição do ponto indicado pelo cursor
 . Impedir borda de trancar o cursor
 . Desenhar cruz central (só com !mouseFree)
 . Impedir câmera de passar do chão
 . Desenhar bússola (Heading) (removível)
 . Desenhar retângulos translúcidos no HUD
 . Barra de espaço jogando pra posição inicial anterior
 - Trocar espaços no início das linhas por tabs (procurar "  " | Geany menu Edit|Preferences|Editor|Display|Show white space)
 - acrescentar luzes (luz não ficou direcional...)
	We can treat osg::Light as a normal rendering attribute, too. For example, applying a
	light object to the root node will also affect its sub-graph. However, there will be an obvious
	difference if we don't make use of light sources. - Beginner's Guide p. 140
 - Inserir objetos 3d (podem representar grandes obras, problemas a resolver, etc) (definir a escala, textura, forma, etc)
 - desenhar shapefiles (ok, mas não serve pra UFs)
 . Identificar os objetos 3d com o mouse
 . Mover os objetos 3d
 . Mostrar nomes das localidades (procurando também nas células vizinhas)
 . Mover com Caps Lock ligado
 . Mostrar nomes das cidades (Voronoi) (ligar e desligar com T)
 . Mostrar e esconder linhas Voronoi com tecla V
 . Colocar todos os ca. 5500 municípios com população
 . Nomes das cidades atrapalham ao apanhar objetos
 . Mostrar nomes de cidades e locais com acentuação (usar UTF-8)
 . Usar outras fontes no HUD (por hora só Charter.ttf)
 . Criar sistema de log (não deve ser usado para informações a cada frame; para debug, usar std::cout)
 . Mostrar tipo de vegetação apontado pelo mouse (cores no mapa não estão 100% fieis, cores espúrias aparecendo -- como lilás em Manaus !!)
 . Ler parâmetros da linha de comando
	. número de células Voronoi
	. velocidade do OVNI
 . Inserir billboards representando hidrelétricas, etc.
 - Criar botões clicáveis no HUD (ligar e desligar bússola, linhas das UFs, etc)
 / Fazer as iterações das interseções baseadas no nome dos objetos e não nas classes
 * trocar Vec3d por Vec3f ou Vec3
 * Mostrar o nome de cada billboard (hidrelétricas...) separadamente. E assim não inseri-los em Locais
 * Inserir modelos 3d
 * Inserir texturas nos modelos 3d
 * Inserir texturas nos cubos
 * Selecionar os cubos/modelos com texturas usando outlines (examples/osgoutline)
 * Selecionar os cubos/modelos com texturas usando linhas brancas nos vértices (examples/osgscribe, Beginner's Guide p. 245: clicking and selecting geometries)
 * Separar o código em classes
	* OVNIController -> OVNIManipulator?
	* Cidades: apresentam um texto em cima (texto preto para capitais?)
	* Locais: aparecem quando passamos o mouse no mapa
	* Estruturas/Locais (pontos) com billboard, como hidrelétricas
	* Estruturas/Locais (linhas) com shapefiles? com decalque?
	* Estruturas/Locais (áreas) com shapefiles? com decalque? (UFs, UCs, TIs...)
	* HUD
 * Tirar cidades menores que X habitantes ??
 * Investigar memory leaks
 * Investigar bottlenecks de desempenho e como melhorá-los (osgUtil::Optimizer ??)
 * Nomes das cidades sumindo atrás do terreno montanhoso (mostrar no HUD com coordenadas globais?)

// https://groups.google.com/forum/#!searchin/osg-users/displaying$20icons|sort:relevance/osg-users/3hXkQ5jXo9I/Vaepg8TrFV8J
	To display a node on top of the other, I think you can set off the depth buffer test and put it into a render bin up all the others (> 10, since it is the transparent bin, if I remember) to draw them at the end. It's all done via StateSet, you should find them easily.
		ou
	To translate objects from object coordinates to screen coordinates you
	must multiple it by a series of matrices in turn, these
	multiplications can all be wrapped up into a single matrices.  The
	order of transformation is:
	  world_coordinate = object_coordinate * world_matrix;
	  eye_coordinate = world_coordinate * view_matrix;
	  clipspace_coordinate = eye_coordinate * projection_matrix;
	  window_coordindates = clipspace_coordinate * viewport_matrix;
	These can be put together thus:
	  window_coordinate = object_coordinate * (world_matrix * view_matrix
	* projection_matrix * viewport_matrix);
	The world_matrix can be obtained from the node->getWorldMatrices()
	which provides a list of matrices, as and Node can have multiple
	parent paths and be in have multiple world transforms placing it in
	space.  The rest of the matrices come from the Camera that you wish to
	get the window coords from, here it'll be camera->getViewMatrix() *
	camera->getProjectionMatrix() *
	camera->getViewport()->computeWindowMatrix();

 * Tamanho máximo dos nomes das cidades não deve ser tão grande (ou mostrar ponto exato da cidade com linha preta vertical)
 * Se 2 ou mais cidades estiverem muito próximas (? km) uma da outra, mostrar só a maior delas
 * Dar nome aos objetos 3d dentro do jogo (osgWidget::Input?)
 * UFs com decalque, overlay, shaders, mudança da textura... ?
 * Mudar o tamanho da janela dentro do jogo
 * Estudar animações (para os objetos 3d e para a intro)
 * Intro: aproximando velozmente do "infinito", desacelerando até a posição inicial sobre o país (osglight.cpp:130)
 * Fazer a velocidade do movimento não depender dos FPS
 * Testar em outro PC, sem OSG instalado (precisa compilar usando -static)
 * Bússola e outros elementos devem escalar com o tamanho da janela? Quanto?
 * Usar mapa com oceano azul
 * Calcular altitude real
 * Botão direito do mouse (?): navegar na mesma altitude
 * Botão direito do mouse travando movimento?
 * mudar cálculos do handle para outro lugar?
 * mudar FirstPersonManipulator para StandardManipulator?
 * criar controlador a pé
 * Usar teclas PageUp para ganhar altitude, e PageDown para perder altitude
 * Usar teclas Shift para acelerar
 * Usar teclas QE para girar ao redor do eixo vertical (do mundo, não da câmera -- pois não há roll). Embora os movimentos horizontais do mouse já façam isso, pode fazer mais rápido, e também pode ser útil no MODO B
 * osgEarth?
 * 
 */

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <osg/AutoTransform>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/ValueObject>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/UpdateMatrixTransform>
#include <osgAnimation/StackedTranslateElement>
#include <osgDB/ReadFile>
#include <osgFX/Outline>
#include <osgGA/FirstPersonManipulator>
//#include <osgSim/OverlayNode>
#include <osgText/Text>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/PrintVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>
#include <sys/stat.h>
#include <sys/types.h>

// https://github.com/pvigier/FortuneAlgorithm - Diagrama de Voronoi
// https://pvigier.github.io/2018/11/18/fortune-algorithm-details.html
#include "pvigier.h"

#include "png.h" // ou usar osg::Image ??

int pngW, pngH;
png_byte color_type;
/*
#define PNG_COLOR_TYPE_GRAY 0
#define PNG_COLOR_TYPE_PALETTE  (PNG_COLOR_MASK_COLOR | PNG_COLOR_MASK_PALETTE)
#define PNG_COLOR_TYPE_RGB        (PNG_COLOR_MASK_COLOR)
#define PNG_COLOR_TYPE_RGB_ALPHA  (PNG_COLOR_MASK_COLOR | PNG_COLOR_MASK_ALPHA)
#define PNG_COLOR_TYPE_GRAY_ALPHA (PNG_COLOR_MASK_ALPHA)
// aliases
#define PNG_COLOR_TYPE_RGBA  PNG_COLOR_TYPE_RGB_ALPHA
#define PNG_COLOR_TYPE_GA  PNG_COLOR_TYPE_GRAY_ALPHA
*/
png_byte bit_depth;
png_bytep *row_pointers = NULL;

/*
	bit_depth
		holds the bit depth of one of the image channels
		valid values are 1, 2, 4, 8, 16 and depend also on the color_type
	color_type
		describes which color/alpha channels are present
		0 PNG_COLOR_TYPE_GRAY
			(bit depths 1, 2, 4, 8, 16)
		1 PNG_COLOR_TYPE_PALETTE
			(bit depths 1, 2, 4, 8)
		2 PNG_COLOR_TYPE_RGB
			(bit_depths 8, 16)
		3 PNG_COLOR_TYPE_RGB_ALPHA
			(bit_depths 8, 16)
		4 PNG_COLOR_TYPE_GRAY_ALPHA
			(bit depths 8, 16)
*/

void read_png_file(const char *filename) {
	FILE *fp = fopen(filename, "rb");
	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if(!png) abort();
	png_infop info = png_create_info_struct(png);
	if(!info) abort();
	if(setjmp(png_jmpbuf(png))) abort();
	png_init_io(png, fp);
	png_read_info(png, info);
	pngW       = png_get_image_width(png, info);
	pngH       = png_get_image_height(png, info);
	color_type = png_get_color_type(png, info);
	bit_depth  = png_get_bit_depth(png, info);
	// Read any color_type into 8bit depth, RGBA format.
	// See http://www.libpng.org/pub/png/libpng-manual.txt
	if(bit_depth == 16)
		png_set_strip_16(png);
	if(color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_palette_to_rgb(png);
	// PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
	if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
		png_set_expand_gray_1_2_4_to_8(png);
	if(png_get_valid(png, info, PNG_INFO_tRNS))
		png_set_tRNS_to_alpha(png);
	// These color_type don't have an alpha channel then fill it with 0xff.
	if(color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
	if(color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
		png_set_gray_to_rgb(png);
	png_read_update_info(png, info);
	if (row_pointers) abort();
	row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * pngH);
	for(int y = 0; y < pngH; y++) {
		row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
	}
	png_read_image(png, row_pointers);
	fclose(fp);
	png_destroy_read_struct(&png, &info, NULL);
}

/*
1	Tree cover, broadleaved, evergreen
2	Tree cover, broadleaved. deciduous closed
3	Tree cover, broadleaved, deciduous, open
4	Tree cover needle-leaved, evergreen
5	Tree cover needleleaved, deciduous
6	Tree cover, mixed leaf type
7	Tree cover, regularly flooded, fresh water
8	Tree cover, regularly flooded saline water
9	Mosaic: Tree Cover / Other natural vegetation
10	Tree Cover, burnt
11	Shrub Cover, closed-open, evergreen
12	Shrub Cover, closed-open, deciduous
13	Herbaceous Cover, closed-open
14	Sparse herbaceous or sparse shrub cover
15	Regularly flooded shrub and/or herbaceous cover
16	Cultivated and managed areas
17	Mosaic: Cropland / Tree Cover / Other natural vegetation
18	Mosaic: Cropland / Shrub and/or grass cover
19	Bare Areas
20	Water Bodies
21	Snow and Ice
22	Artificial surfaces and associated areas
23	No Data
*/
std::string biomas[] = {"Floresta ombrófila sempre verde","Floresta ombrófila decídua","Floresta temperada/mista","Floresta inundável (água doce)/várzea/igapó",
						"Floresta inundável (água salgada)/mangue","Árvores/outra vegetação natural","Cobertura arbustiva","Cobertura herbácea",
						"Cobertura arbustiva/herbácea esparsa","Cobertura arbustiva/herbácea inundável","Área cultivada","Cultivo/árvores/outra vegetação natural",
						"Cultivo/arbustos/gramíneas","Solo nu (areia/gelo/neve)","Água","Área urbana","Árvores queimadas"};
std::string rgb[] = {"79ff79","caff4c","6bc3c8","46ac46",
					"ba69c0","c3834a","ffc549","ffddce",
					"fff9a5","c1c1ff","ffffff","ba875a",
					"c4ba69","d2d2d2","7979ff","444444","ff0000"};
					//"c4ba69","d2d2d2","69acff","505050","ff0000"};

std::string get_png_pixel(int x, int y)
{
	png_bytep row = row_pointers[y];
	if (color_type == 2 && bit_depth == 8)
	{
		png_bytep px = &(row[x * 4]); // se não tem alpha, pq 4 ??
		std::stringstream R, G, B;
		R << std::setfill('0') << std::setw(2) << std::hex << (int)px[0];
		G << std::setfill('0') << std::setw(2) << std::hex << (int)px[1];
		B << std::setfill('0') << std::setw(2) << std::hex << (int)px[2];
		std::string RGB = R.str() + G.str() + B.str();
		std::string* rgbitr = std::find(rgb,rgb+17,RGB);
		if (rgbitr != rgb+17)
			return biomas[std::distance(rgb,rgbitr)];
		else
			return "";
	}
	return "";
}

/* Itens de interesse nos locais

table(loc$nm_classe[which(loc$tp_geom == 'Ponto')])
74		barragem
27		capital
102		complexo_portuario
1177	elemento_fisiografico_natural ??
373		est_gerad_energia_eletrica
2788	ext_mineral
155		foz_maritima
310		hidreletrica
85		pico
1478	pista_ponto_pouso ?
98		subest_transm_distrib_energia_eletrica
63		termeletrica

table(loc$nm_classe[which(loc$tp_geom == 'Linha')]) // converter pra shapefile? usar centróide?
778		barragem
2551	elemento_fisiografico_natural ??
122		foz_maritima
609635	trecho_drenagem (existe shapefile próprio)
760		trecho_energia
1842	trecho_ferroviario
15058	trecho_hidroviario
44407	trecho_rodoviario

table(loc$nm_classe[which(loc$tp_geom == 'Area')]) // shp = shapefiles (provavelmente melhores) disponíveis
507		ext_mineral					shp
1		foz_maritima
2164	ilha
7687	massa_dagua
444		terra_indigena				shp
202		terreno_sujeito_inundacao
321		unidade_protecao_integral	shp
943		unidade_protegida			shp
555		unidade_uso_sustentavel		shp
*/

std::string logFileName;

void log( std::string msg )
{
	time_t t = time(nullptr);
	struct tm t_tm = *std::localtime(&t);
	std::ofstream outfile(logFileName.c_str(), std::ios_base::app);
	outfile << std::put_time(&t_tm, "%H:%M:%S ") << msg << "\n";
}

/* create an ordinary camera node which will be rendered on the top after
the main scene is drawn. It can be used to display some heads-up display (HUD) texts
and images. */
osg::Camera* createHUDCamera( double left, double right, double bottom, double top )
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
	camera->setClearMask( GL_DEPTH_BUFFER_BIT );
	camera->setRenderOrder( osg::Camera::POST_RENDER );
	camera->setAllowEventFocus( false );
	camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top) );
	//camera->setProjectionMatrixAsOrtho( left, right, bottom, top, 100, -100 ); // não funcionou. Os valores Z (texto e menu do HUD) precisam ficar entre 0 e -1
	camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF ); // necessário aqui ??
	camera->setName("cameraHUD");
	return camera.release();
}

// function for creating HUD texts
osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("Charter.ttf"); // https://practicaltypography.com/charter.html
osgText::Text* createText( const osg::Vec3& pos, const std::string& content, float size )
{
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setDataVariance( osg::Object::DYNAMIC );
	text->setFont( g_font.get() );
	text->setCharacterSize( size );
	text->setAxisAlignment( osgText::TextBase::XY_PLANE );
	text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_RIGHT);
	text->setPosition( pos );
	text->setText( content );
	return text.release();
}

class OVNIController : public osgGA::FirstPersonManipulator
{
public:
	OVNIController(osgViewer::Viewer *inputViewer) : _viewer(inputViewer)
	{
		// Start frame timer
		mainTimer.setStartTick();
		setAllowThrow(false); // não continua movendo ao soltar o botão do mouse
	}
	// Override handle function for keyboard input
	virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &);
	virtual std::string pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
	virtual float pickDown( osg::Vec3d pos );
	virtual bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
	virtual bool handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
	virtual bool performMovement();
	virtual bool handleMouseMove( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
protected:
	osgViewer::Viewer *_viewer;
	osg::Timer mainTimer;
};

struct tipop
{
	float lat, lon;
	std::string nome;
};
typedef std::vector<tipop> tipoP;
typedef std::map<int,tipoP> tipoLons;
typedef std::map<int,tipoLons> tipoLats;
tipoLats Locais;	// nomes das localidades
struct tCidade
{
	std::string nome, UF;
	float lat, lon, X;
	uint32_t pop;
};
typedef std::vector<tCidade> tCidades;
tCidades Cidades;

//osg::PI = 3.14159265358979323846;
const float pi = 3.141592653;
float lonMin=-74, lonMax=-34.8, latMin=-33.8666667, latMax=5.3333333;
float lonC = (lonMin+lonMax)/2; // -54.4
float latC = (latMin+latMax)/2; // -14.2666667
float rCeu = (lonMax-lonMin)*2; // 78.4 = "raio do céu" (para calcular a posição inicial da câmera e a posição do "Sol")
float solLO=pi/4, solNS=pi/8; // ângulo do Sol, leste-oeste e norte-sul
osg::Vec3d posSol = osg::Vec3d( lonC+rCeu*sin(solLO), latC+rCeu*sin(solNS), rCeu*cos(solLO)*cos(solNS) );
osg::ref_ptr<OVNIController> controller;
const float acc = 0.1;
const float maxVel = 1;
osg::Vec3d camSpeed;
float velX=0, velY=0, velZ=0, velSol=0.01;	// XYZ: velocidades lateral, vertical e frontal
float camAlt = 0, camLat = 0, camLon = 0, ptrAlt = 0, ptrLat = 0, ptrLon = 0;
bool keyW=false, keyA=false, keyS=false, keyD=false, keyR=false, keyF=false, keyL=false, keyO=false, keyK=false, keyI=false;
bool Homing=false, mouseFree=false, Warping=false, mostraBussola=true, movendoBox=false;
const float inputTimeInterval = 0.02;
double maxTick = inputTimeInterval;
int windowX, windowY, windowW, windowH;
osgText::Text* text;
osgViewer::Viewer::Windows::iterator window;
osg::ref_ptr<osg::Node> Sol;
osg::ref_ptr<osg::Geode> crossGeode;
osg::ref_ptr<osg::PositionAttitudeTransform> bussola;
osg::ref_ptr<osg::PositionAttitudeTransform> menu;
osg::ref_ptr<osg::Group> blocos;
osg::ref_ptr<osg::Group> grpVoronoi;
osg::ref_ptr<osg::Group> grpCidades;
osg::ref_ptr<osg::Group> grpHidreletrica;
osg::ref_ptr<osg::Group> grpIndigena;
osg::Geode* selecionado = nullptr;
osg::ref_ptr<osg::Billboard> geodeHidreletrica;
osg::ref_ptr<osg::Billboard> geodeIndigena;
//osg::ref_ptr<osg::MatrixTransform> mtHidreletrica;
osgUtil::PrintVisitor pv( std::cout );
int nVoronoi = 200;
float ovniSpeed = 10;
osg::ref_ptr<osg::Geode> botaoHover;
bool hoverHidreletrica = false;
bool hoverIndigena = false;

bool FileExists(std::string strFilename)
{
	struct stat stFileInfo;
	return (stat(strFilename.c_str(),&stFileInfo) == 0);
}

// cubo com textura - https://groups.google.com/forum/#!searchin/osg-users/textured$20cube|sort:date/osg-users/8x7LdKUQlRM/008IWsokAwAJ

/*osg::Geometry* criaCubo() {
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array();
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

	// ------ top (front?)
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 1.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f,-1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f,-1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f,-1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f,-1.0f, 0.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	// ------ top (front?)
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 1.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 1.0f));
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	// ------ back
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 1.0f));
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 1.0f));
	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	// ------ Bottom
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f));
	normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	// ------ Left
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 1.0f, 1.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(-1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(-1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(-1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(-1.0f, 0.0f, 0.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	// ------ Right
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 1.0f));
	vertices->push_back(osg::Vec3(1.0f, 0.0f, 1.0f));
	normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
	texcoords->push_back( osg::Vec2(0.0f, 0.0f) );
	texcoords->push_back( osg::Vec2(0.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 1.0f) );
	texcoords->push_back( osg::Vec2(1.0f, 0.0f) );

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(vertices);
	geom->setNormalArray(normals, osg::Array::Binding::BIND_PER_VERTEX);
	geom->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 24));
	geom->setTexCoordArray(0, texcoords.get());
	osgUtil::SmoothingVisitor::smooth(*geom);

	geom->setTexCoordArray(0, texcoords.get(), osg::Array::Binding::BIND_PER_VERTEX);

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	osg::ref_ptr<osg::Image> image = osgDB::readImageFile("texture/1K-wood_parquet_2/1K-wood_parquet_2-diffuse.jpg");
	texture->setImage(image);
	texture->setUnRefImageDataAfterApply(true);
	
	return geom.release();
}*/

// cria textura para Hidrelétricas, ...
float escalaIcone = 0.2; // com imagem "icons/*.png" de 512x512 pixels

osg::Geometry* createQuad(const char* fName, const char* name)
{
	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	osg::ref_ptr<osg::Image> image = osgDB::readImageFile( fName );
	texture->setImage( image.get() ); // qual a diferença entre image e image.get() aqui ?? Ambos funcionam
	osg::ref_ptr<osg::Geometry> quad = osg::createTexturedQuadGeometry(
		osg::Vec3(-escalaIcone/2, 0.0f,-escalaIcone/2), // -metade da largura e altura (abaixo)
		osg::Vec3(escalaIcone,0.0f,0.0f),		// largura
		osg::Vec3(0.0f,0.0f,escalaIcone) );		// altura
	osg::StateSet* ss = quad->getOrCreateStateSet();
	ss->setTextureAttributeAndModes( 0, texture.get() );
	ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	quad->setName(name);
	return quad.release();
}

void getLocais(void)
{
	// Limpa locais existentes (para quando mudar de país)
	if (Locais.begin() != Locais.end())
	{
		tipoLats::iterator iC;
		for (iC=Locais.begin();iC!=Locais.end();iC++)
		{
			tipoLons Lons = iC->second;
			if (Lons.begin() != Lons.end())
			{
				tipoLons::iterator iL;
				for (iL=Lons.begin();iL!=Lons.end();iL++)
				{
					tipoP P = iL->second;
					if (P.begin() != P.end())
						P.clear();
				}
				Lons.clear();
			}
		}
		Locais.clear();
	}
	// https://www.ibge.gov.br/geociencias/cartas-e-mapas/bases-cartograficas-continuas/15759-brasil.html?=&t=downloads (bc250_nomesgeograficos_2019-10-29.csv, apenas "Pontos" do Brasil)
	if (FileExists("data/locais_br_2019.csv"))
	{
		std::ifstream flocais(std::string("data/locais_br_2019.csv").c_str());
		if (flocais.is_open())
		{
			std::string linha, nome, nomeCateg, nomeClasse, uf;
			size_t pos;
			float lat, lon;
			int intlat, intlon, numLocais=0;
			tipop p;
			tipoP* P;
			tipoP::iterator iP;
			tipoLons* pLons;
			tipoLons::iterator iLons;
			tipoLats::iterator iLats;

			// cria símbolos das hidrelétricas, terras indígenas...
			osg::Geometry* quadHidreletrica = createQuad("icons/hidreletrica.png","QuadHidrelétrica");
			osg::Geometry* quadIndigena = createQuad("icons/indigena.png","QuadIndígena");

			std::getline(flocais,linha); // pula a linha de título
			while (!flocais.eof())
			{
				std::getline(flocais,linha);

				pos = linha.find('\t');
				nome = linha.substr(0,pos);
				linha = linha.substr(pos+1);
				
				pos = linha.find('\t');
				lat = atof(linha.substr(0,pos).c_str());
				linha = linha.substr(pos+1);

				pos = linha.find('\t');
				lon = atof(linha.substr(0,pos).c_str());
				linha = linha.substr(pos+1);

				pos = linha.find('\t');
				nomeCateg = linha.substr(0,pos);
				linha = linha.substr(pos+1);

				pos = linha.find('\t');
				nomeClasse = linha.substr(0,pos);
				uf = linha.substr(pos+1);
				
				if( nomeClasse == "hidreletrica" )
				{
					geodeHidreletrica = new osg::Billboard;
					geodeHidreletrica->setMode( osg::Billboard::POINT_ROT_EYE );
					geodeHidreletrica->setName(nome);
					geodeHidreletrica->addDrawable( quadHidreletrica, osg::Vec3( lon, lat, controller->pickDown( osg::Vec3d(lon,lat,1) )+escalaIcone/2 ) );
					osg::StateSet* ss = geodeHidreletrica->getOrCreateStateSet();
					ss->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
					grpHidreletrica->addChild( geodeHidreletrica );
				}
				else
				if( nomeClasse == "terra_indigena" )
				{
					geodeIndigena = new osg::Billboard;
					geodeIndigena->setMode( osg::Billboard::POINT_ROT_EYE );
					geodeIndigena->setName(nome);
					geodeIndigena->addDrawable( quadIndigena, osg::Vec3( lon, lat, controller->pickDown( osg::Vec3d(lon,lat,1) )+escalaIcone/2 ) );
					osg::StateSet* ss = geodeIndigena->getOrCreateStateSet();
					ss->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
					grpIndigena->addChild( geodeIndigena );
				}
				
				// insere local nos mapas de busca
				intlat = floor(lat);
				intlon = floor(lon);
				// search Latitude
				iLats = Locais.find(intlat);
				if (iLats != Locais.end())
				{
					// found Latitude, search Longitutde
					pLons = &iLats->second; // all longitudes inside latitude (intlat)
					iLons = pLons->find(intlon);
					if (iLons != pLons->end())
					{
						// found Longitude, insert point
						P = &iLons->second; // longitude (intlon) with latitude (intlat)
						p.lat = lat;
						p.lon = lon;
						p.nome = nome;
						P->push_back(p);
						numLocais++;
					}
					else
					{ // iLons == pLons->end()
						// insert point
						p.lat = lat;
						p.lon = lon;
						p.nome = nome;
						P = new tipoP;
						P->push_back(p);
						// add Longitude
						pLons->insert(std::pair<int,tipoP>(intlon,*P)); // insere nova longitude
						numLocais++;
					}
				}
				else
				{ // iLats == Locais.end()
					// insert point
					p.lat = lat;
					p.lon = lon;
					p.nome = nome;
					P = new tipoP;
					P->push_back(p);
					// add Longitude
					pLons = new tipoLons;
					pLons->insert(std::pair<int,tipoP>(intlon,*P));
					// add Latitude
					Locais.insert(std::pair<int,tipoLons>(intlat,*pLons)); // insere nova latitude
					numLocais++;
				}
			}
		}
	}
}

void getCidades(void)
{
	// Limpa cidades existentes (para quando mudar de país)
	if (Cidades.begin() != Cidades.end())
	{
		Cidades.clear();
	}
	if (FileExists("data/municipios_br_2019.csv"))
	{
		std::ifstream fcidades(std::string("data/municipios_br_2019.csv").c_str());
		if (fcidades.is_open())
		{
			std::string linha, nome;
			size_t pos;
			tCidade cidade;
			float lat, lon;
			int intlat, intlon, numCidades=0;
			tipop p;
			tipoP* P;
			tipoP::iterator iP;
			tipoLons* pLons;
			tipoLons::iterator iLons;
			tipoLats::iterator iLats;
			std::getline(fcidades,linha); // pula a linha de título
			while (!fcidades.eof())
			{
				std::getline(fcidades,linha);

				pos = linha.find("\t");
				cidade.nome = linha.substr(0,pos);
				linha = linha.substr(pos+1);

				pos = linha.find("\t");
				cidade.UF = linha.substr(0,pos);
				linha = linha.substr(pos+1);

				pos = linha.find("\t");
				cidade.lat = atof(linha.substr(0,pos).c_str());
				linha = linha.substr(pos+1);

				pos = linha.find("\t");
				cidade.lon = atof(linha.substr(0,pos).c_str());
				linha = linha.substr(pos+1);

				pos = linha.find("\t");
				cidade.pop = atoi(linha.substr(0,pos).c_str()); // população em 2019

				Cidades.push_back(cidade);
			}
		}
	}
}

osg::Vec3d fromQuat(const osg::Quat& quat, bool degrees)
{
	// From: http://guardian.curtin.edu.au/cga/faq/angles.html
	// Except OSG exchanges pitch & roll from what is listed on that page
	double qx = quat.x();
	double qy = quat.y();
	double qz = quat.z();
	double qw = quat.w();

	double sqx = qx * qx;
	double sqy = qy * qy;
	double sqz = qz * qz;
	double sqw = qw * qw;

	double term1 = 2 * (qx*qy + qw*qz);
	double term2 = sqw + sqx - sqy - sqz;
	//double term3 = -2 * (qx*qz - qw*qy);
	double term4 = 2 * (qw*qx + qy*qz);
	double term5 = sqw - sqx - sqy + sqz;

	double heading = atan2(term1, term2);
	double pitch = atan2(term4, term5);
	//double roll = asin(term3);

	//Return values in degrees if requested, else its radians
	if (degrees)
	{
		heading = osg::RadiansToDegrees(heading);
		pitch = osg::RadiansToDegrees(pitch);
		//roll = osg::RadiansToDegrees(roll);
	}
	return osg::Vec3d(heading, pitch, 0); // (não usa roll)
}

void criaBloco( osg::Quat camRotation ) // ver examples/osgscribe/osgscribe.cpp para exemplos de linhas sobre superfícies
{
	osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable( new osg::Box );
	shape->setDataVariance( osg::Object::DYNAMIC );
	shape->setColor( osg::Vec4(1,0,0,1) ); // vermelho
	//shape->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON ); // deixa translúcido
	shape->setUseDisplayList( false );
	shape->setName( "Box " + std::to_string( blocos->getNumChildren() ) );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Vec3 hpr = fromQuat(camRotation,false); // ângulos da câmera, em radianos
	// hpr.x() = heading , hpr.y = pitch
	geode->setUserValue( "speed", osg::Vec3d(-0.1*sin(hpr.x())*sin(hpr.y()),0.1*cos(hpr.x())*sin(hpr.y()),-0.005) ); // velocidade "pra frente" (câmera) e pra baixo (gravidade)
	geode->setName( "GeodeBox " + std::to_string( blocos->getNumChildren() ) );
	geode->addChild( shape );

	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	trans->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) *
						osg::Matrix::translate( osg::Vec3(camLon, camLat, camAlt)+camRotation*osg::Vec3(0,0,-1) ) ); // posição: -1 = distância à frente da câmera
	trans->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
	trans->getOrCreateStateSet()->setAttributeAndModes( new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
	trans->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	trans->setName("MatrTransfBox " + std::to_string( blocos->getNumChildren() ) );
	trans->addChild( geode );

	blocos->addChild( trans );
	//blocos->accept( pv );
}

bool OVNIController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
		return handleMousePush( ea, aa );
	else
	if( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
		return handleMouseRelease( ea, aa );
	else
	if( ea.getEventType() == osgGA::GUIEventAdapter::MOVE ||
		ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
		return handleMouseMove( ea, aa ); // barra de espaços parou de levar pra coordenada inicial anterior
	if (!_viewer)
		return false;
	// Set the viewer's "eye" position, which is located at the center of the camera.
	osg::Vec3d eyePos;
	osg::Matrix matrix = _viewer->getCameraManipulator()->getMatrix();
	eyePos = matrix.getTrans();
	osg::Quat camRotation = matrix.getRotate();
	// Movement of the camera correlates with W, A, S, D
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
	{
		//std::cout << ea.getKey() << "\n";
		switch(ea.getKey())
		{
			case ' ': // Barra de espaço
				criaBloco( camRotation );
				break;
			case 65289: // Tab
				mouseFree = !mouseFree;
				(*window)->useCursor(mouseFree);
				if (mouseFree)
				{
					crossGeode->setNodeMask(0); // esconde a "mira"
					menu->setNodeMask(1); // mostra o menu
					aa.requestWarpPointer( windowW/2, windowH/2 ); // põe o mouse no meio da tela
				}
				else
				{
					crossGeode->setNodeMask(1); // mostra a "mira"
					menu->setNodeMask(0); // esconde o menu
				}
				break;
			case 65360: // Home
				Homing = true;
				break;
			case 'b': // Bússola
			case 'B':
				mostraBussola = !mostraBussola;
				if (mostraBussola)
				{
					bussola->setNodeMask(1); // mostra a bússola
				}
				else
				{
					bussola->setNodeMask(0); // esconde a bússola
				}
				break;
			case 't': // Nomes das cidades
			case 'T':
				grpCidades->setNodeMask(!grpCidades->getNodeMask());
				break;
			case 'v': // Linhas Voronoi
			case 'V':
				grpVoronoi->setNodeMask(!grpVoronoi->getNodeMask());
				break;
			case 'w':
			case 'W':
				keyW = true;
				break;
			case 'a':
			case 'A':
				keyA = true;
				break;
			case 's':
			case 'S':
				keyS = true;
				break;
			case 'd':
			case 'D':
				keyD = true;
				break;
			case 'r':
			case 'R':
				keyR = true;
				break;
			case 'f':
			case 'F':
				keyF = true;
				break;
			case 'l':
			case 'L':
				keyL = true;
				break;
			case 'o':
			case 'O':
				keyO = true;
				break;
			case 'k':
			case 'K':
				keyK = true;
				break;
			case 'i':
			case 'I':
				keyI = true;
				break;
		}
	}
	else
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
	{
		switch(ea.getKey())
		{
			case 'w':
			case 'W':
				keyW = false;
				break;
			case 'a':
			case 'A':
				keyA = false;
				break;
			case 's':
			case 'S':
				keyS = false;
				break;
			case 'd':
			case 'D':
				keyD = false;
				break;
			case 'r':
			case 'R':
				keyR = false;
				break;
			case 'f':
			case 'F':
				keyF = false;
				break;
			case 'l':
			case 'L':
				keyL = false;
				break;
			case 'o':
			case 'O':
				keyO = false;
				break;
			case 'k':
			case 'K':
				keyK = false;
				break;
			case 'i':
			case 'I':
				keyI = false;
				break;
		}
	}
	osg::Vec3 hpr = fromQuat(camRotation,true); // ângulos da câmera, em graus
	std::stringstream eyeX, eyeY, eyeZ;

	camLat = eyePos.y();
	camLon = eyePos.x();
	float chao = pickDown( eyePos );
	if( chao > 0 && eyePos.z() < chao+0.2 ) // altura mínima do OVNI
	{
		eyePos = osg::Vec3( camLon, camLat, chao+0.2 );
	}
	camAlt = eyePos.z();
	eyeY << std::fixed << std::setprecision(1) << camLat;
	eyeX << std::fixed << std::setprecision(1) << camLon;
	eyeZ << std::fixed << std::setprecision(2) << camAlt;

	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	std::string pickStr="";
	if (view)
	{
		if (mouseFree)
			pickStr = pick(view,ea); // coordenadas do mouse
		else
		{
			osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter(ea);
			event->setX(windowW/2);
			event->setY(windowH/2);
			pickStr = pick(view,*event); // coordenadas do centro da tela
		}
	}
	bussola->setAttitude(osg::Quat(osg::inDegrees(-hpr.x()),osg::Vec3d(0,0,1)));
	text->setText( "OVNI lat:lon:alt " + eyeY.str() + ":" + eyeX.str() + ":" + eyeZ.str() + "\n" + pickStr, osgText::String::ENCODING_UTF8 );

	eyePos += camRotation * camSpeed * ovniSpeed/10 * camAlt/20; // * camAlt/10 faz a velocidade aumentar com a altitude
	matrix.setTrans(eyePos);
	// Check [mainTimer.time % (time divisor) == 0]
	if (mainTimer.time_s() >= maxTick) // ??
	{
		_viewer->getCameraManipulator()->setByMatrix(matrix); // muda posição, não o ângulo
		maxTick += inputTimeInterval;
	}

	return false;
}

std::string getLocMaisPerto(tipoP P, float x, float y)
{
	float dist, menordist = 1e10;
	tipoP::iterator iP, menorP;
	tipop p;
	for (iP=P.begin(); iP<P.end(); iP++)
	{
		p = *iP;
		dist = sqrt(pow(x-p.lon,2)+pow(y-p.lat,2));
		if (dist < menordist)
		{
			menordist = dist;
			menorP = iP;
		}
	}
	p = *menorP;
	return p.nome;
}

std::string getLocMaisPerto2(tipoP P[8], float x, float y)
{
	float dist, menordist = 1e10;
	tipoP::iterator iP, menorP;
	tipop p;
	for (int i=0; i<8; i++)
		for (iP=P[i].begin(); iP<P[i].end(); iP++)
		{
			p = *iP;
			dist = sqrt(pow(x-p.lon,2)+pow(y-p.lat,2));
			if (dist < menordist)
			{
				menordist = dist;
				menorP = iP;
			}
		}
	if (menordist < 1e10)
	{
		p = *menorP;
		return p.nome;
	}
	else
		return "Indefinido";
}

// examples/osgpick/osgpick.cpp:102
std::string OVNIController::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	if (view->computeIntersections(ea,intersections))
	{
		std::stringstream ptrX, ptrY, ptrZ;
		if( movendoBox && selecionado ) // arrastando algum Box (não precisa do "selecionado", mas por segurança...)
		{
			for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
				hitr != intersections.end();
				++hitr)
			{
				if( strcmp( hitr->drawable->className(),"Geometry" ) == 0 ) // terreno
				{
					osg::MatrixTransform* mtxTr = dynamic_cast<osg::MatrixTransform*>(selecionado->asNode()->getParent(0));
					if( mtxTr )
					{
						osg::Vec3d pos = hitr->getWorldIntersectPoint();
						ptrX << std::fixed << std::setprecision(1) << pos.x();
						ptrY << std::fixed << std::setprecision(1) << pos.y();
						ptrZ << std::fixed << std::setprecision(2) << pos.z();
						pos.z() += 0.05;
						mtxTr->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
						return "Movendo para lat:lon:alt: " + ptrX.str() + ":" + ptrY.str() + ":" + ptrZ.str();
					}
				}
			}
			return "";
		}
		else // não está arrastando nenhum Box
		{
			hoverHidreletrica = false;
			hoverIndigena = false;
			osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
			while( hitr != intersections.end() && hitr->drawable->getName().compare("GeometryMenu") != 0 ) // procura menu
			{
				hitr++;
			}
			if( hitr != intersections.end() ) // chegou no menu
			{
				while( hitr != intersections.end() && hitr->drawable->getName().compare("GeometryBotao") != 0 ) // procura botões do menu
				{
					hitr++;
				}
				if( hitr != intersections.end() ) // chegou num botão
				{
					osg::NodePath path = hitr->nodePath;
					for( osg::NodePath::const_iterator hitNode = path.begin(); hitNode != path.end(); ++hitNode)
					{
						if( strcmp( (*hitNode)->className(), "Geode" ) == 0 )
						{
							if ( (*hitNode)->getName().compare("Hidrelétrica") == 0 )
							{
								hoverHidreletrica = true;
								botaoHover = dynamic_cast<osg::Geode*>(*hitNode);
								return "Botão: Hidrelétrica";
							}
							else
							if ( (*hitNode)->getName().compare("Terra Indígena") == 0 )
							{
								hoverIndigena = true;
								botaoHover = dynamic_cast<osg::Geode*>(*hitNode);
								return "Botão: Terra Indígena";
							}
						}
					}
					return "Botão não identificado";
				}
				return "Menu";
			}
			// se não for menu, olha de novo as interseções, do início
			hitr = intersections.begin();
			while( hitr != intersections.end() &&
					(strcmp( hitr->drawable->className(),"Text") == 0 || hitr->drawable->getName().compare("GeometryBussola") == 0) ) // pula textos e bússola
			{
				hitr++;
				/*if( hitr != intersections.end() )
				{
					std::cout << "2intersect: " << hitr->drawable->getName() << " (" << hitr->drawable->className() << ")\n";
					hitr->drawable->accept( pv );
				}*/
			}
			if( hitr == intersections.end() )
				return "";
			int pngX, pngY;
			std::string bioma;
			ptrLat = hitr->getWorldIntersectPoint().y();
			ptrLon = hitr->getWorldIntersectPoint().x();
			ptrAlt = hitr->getWorldIntersectPoint().z();
			pngX = pngW*(ptrLon-lonMin)/(lonMax-lonMin);
			pngY = pngH*(latMax-ptrLat)/(latMax-latMin);
			bioma = get_png_pixel(pngX,pngY);
			ptrX << std::fixed << std::setprecision(1) << ptrLon;
			ptrY << std::fixed << std::setprecision(1) << ptrLat;
			ptrZ << std::fixed << std::setprecision(2) << ptrAlt;
			osg::NodePath path = hitr->nodePath;
			bool achou = false;
			std::string nome = "";
			for( osg::NodePath::const_iterator hitNode = path.begin(); hitNode != path.end(); ++hitNode)
			{
				//std::cout << "hitNode: " << (*hitNode)->getName() << " (" << (*hitNode)->className() << ")\n";
				if( strcmp( (*hitNode)->className(),"Geode" ) == 0 || strcmp( (*hitNode)->className(),"Terrain" ) == 0 ||
					strcmp( (*hitNode)->className(),"Billboard" ) == 0 )
				{
					nome = (*hitNode)->getName();
					achou = true;
					osg::ShapeDrawable* shpDrb;
					if ( strcmp( (*hitNode)->className(),"Geode" ) == 0 )
					{
						shpDrb = dynamic_cast<osg::ShapeDrawable*>((*hitNode)->asGroup()->getChild(0));
						if( selecionado == nullptr )
						{
							if (shpDrb)
								shpDrb->setColor( osg::Vec4(1,1,1,1) );
							selecionado = (*hitNode)->asGeode();
						}
						else
						{
							if( selecionado->getName().compare( (*hitNode)->getName() ) != 0 )
							{
								if (shpDrb)
									shpDrb->setColor( osg::Vec4(1,1,1,1) );
								shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->asGroup()->getChild(0));
								if (shpDrb)
									shpDrb->setColor( osg::Vec4(1,0,0,1) );
								selecionado = (*hitNode)->asGeode();
							}
						}
					}
					else // Terrain ou Billboard
					{
						if( selecionado != nullptr)
						{
							shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->asGroup()->getChild(0));
							if (shpDrb)
								shpDrb->setColor( osg::Vec4(1,0,0,1) );
							selecionado = nullptr;
						}
					}
					break;
				}
			}

			// local mais próximo
			float dist, menordist;
			int intlon, intlat;
			tipoLons pLons;
			tipoLons::iterator iLons;
			tipoLats::iterator iLats;
			std::string nomeLoc;
			intlon = floor(ptrLon);
			intlat = floor(ptrLat);
			iLats = Locais.find(intlat);
			if (iLats != Locais.end()) // deve olhar em mais de uma quadrícula ao mesmo tempo, quando o mouse estiver na beira de uma !!
			{
				pLons = iLats->second;
				iLons = pLons.find(intlon);
				if (iLons != pLons.end())
					nomeLoc = "Local: " + getLocMaisPerto(iLons->second,ptrLon,ptrLat);
				else
				{
					// nenhum ponto nessa quadrícula (1° x 1°) - procura nas quadrículas ao redor
					tipoP P[8];
					if (Locais.find(intlat-1) != Locais.end())
					{
						if (Locais.find(intlat-1)->second.find(intlon-1) != Locais.find(intlat-1)->second.end())
							P[0] = Locais.find(intlat-1)->second.find(intlon-1)->second; // copia os dados ou passa por referência ??
						if (Locais.find(intlat-1)->second.find(intlon) != Locais.find(intlat-1)->second.end())
							P[1] = Locais.find(intlat-1)->second.find(intlon)->second;
						if (Locais.find(intlat-1)->second.find(intlon+1) != Locais.find(intlat-1)->second.end())
							P[2] = Locais.find(intlat-1)->second.find(intlon+1)->second;
					}
					if (Locais.find(intlat)->second.find(intlon-1) != Locais.find(intlat)->second.end())
						P[3] = Locais.find(intlat)->second.find(intlon-1)->second;
					if (Locais.find(intlat)->second.find(intlon+1) != Locais.find(intlat)->second.end())
						P[4] = Locais.find(intlat)->second.find(intlon+1)->second;
					if (Locais.find(intlat+1) != Locais.end())
					{
						if (Locais.find(intlat+1)->second.find(intlon-1) != Locais.find(intlat+1)->second.end())
							P[5] = Locais.find(intlat+1)->second.find(intlon-1)->second;
						if (Locais.find(intlat+1)->second.find(intlon) != Locais.find(intlat+1)->second.end())
							P[6] = Locais.find(intlat+1)->second.find(intlon)->second;
						if (Locais.find(intlat+1)->second.find(intlon+1) != Locais.find(intlat+1)->second.end())
							P[7] = Locais.find(intlat+1)->second.find(intlon+1)->second;
					}
					nomeLoc = "Local: " + getLocMaisPerto2(P,ptrLon,ptrLat); // está copiando os dados de P ou passando por referência ??
				}
			}
			else
				nomeLoc = "Local: Indefinido";
			if( achou )
				return "Cursor lat:lon:alt: " + ptrY.str() + ":" + ptrX.str() + ":" + ptrZ.str() + "\n" + nomeLoc + "\n" + bioma + " (" + nome + ")";
			else
				return "Cursor lat:lon:alt: " + ptrY.str() + ":" + ptrX.str() + ":" + ptrZ.str() + "\n" + nomeLoc + "\n" + bioma;
		}
	}
	else
		return "";
}

// https://groups.google.com/forum/#!searchin/osg-users/LineSegmentIntersector|sort:date/osg-users/f4WZnzr8X5w/37pAZ4QqAwAJ
// src/osgUtil/LineSegmentIntersector.cpp
float OVNIController::pickDown( osg::Vec3d pos ) // retorna altitude do terreno (className() == "Geometry") naquele ponto, ou -1 se estiver fora do terreno
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	osg::Vec3d pos1( pos.x(), pos.y(), pos.z()+100 );
	osg::Vec3d pos2( pos.x(), pos.y(), -100 );
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intsec = new osgUtil::LineSegmentIntersector( osgUtil::Intersector::MODEL, pos1, pos2 );
	osgUtil::IntersectionVisitor iv( intsec.get() );
	_viewer->getSceneData()->accept( iv );
	if( intsec->containsIntersections() )
	{
		for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intsec->getIntersections().begin();
			hitr != intsec->getIntersections().end();
			++hitr)
		{
			if ( strcmp(hitr->drawable->className(),"Geometry") == 0 )
			{
				return hitr->getWorldIntersectPoint().z();
			}
		}
		return -1;
	}
	else
		return -1;
}

// apertou o botão do mouse
bool OVNIController::handleMousePush(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	flushMouseEventStack();
	addMouseEvent( ea );
	unsigned int buttonMask = _ga_t0->getButtonMask();
	if( hoverHidreletrica && buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) // clicou no botão da hidrelétrica
	{
		grpHidreletrica->setNodeMask(!grpHidreletrica->getNodeMask());
		botaoHover->getOrCreateStateSet()->setMode(GL_BLEND, !botaoHover->getOrCreateStateSet()->getMode(GL_BLEND)); // inverte a transparência
	}
	else
	if( hoverIndigena && buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) // clicou no botão da terra indígena
	{
		grpIndigena->setNodeMask(!grpIndigena->getNodeMask());
		botaoHover->getOrCreateStateSet()->setMode(GL_BLEND, !botaoHover->getOrCreateStateSet()->getMode(GL_BLEND)); // inverte a transparência
	}
	else
	if( selecionado && mouseFree )
	{
		osg::ShapeDrawable* shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->getChild(0));
		if (shpDrb)
		{
			if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
			{
				shpDrb->setColor( osg::Vec4(1,1,0,1) ); // amarelo
				movendoBox = true;
			}
		}
	}
	return false;
}

// soltou o botão do mouse
bool OVNIController::handleMouseRelease(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	flushMouseEventStack();
	addMouseEvent( ea );
	if( selecionado )
	{
		osg::ShapeDrawable* shpDrb = dynamic_cast<osg::ShapeDrawable*>(selecionado->getChild(0));
		if (shpDrb)
		{
			shpDrb->setColor( osg::Vec4(1,1,1,1) ); // branco
			movendoBox = false;
		}
	}
	return false;
}

bool OVNIController::performMovement()
{
	// return if less then two events have been added
	if( _ga_t0.get() == NULL || _ga_t1.get() == NULL )
		return false;
	if( mouseFree )
	{
		if( Warping )
			mouseFree = false;
		return false;
	}
	if( Warping )
		Warping = false;
	// get delta time
	double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
	if( eventTimeDelta < 0. )
	{
		OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
		eventTimeDelta = 0.;
	}
	// get deltaX and deltaY
	float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
	float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();
	// return if there is no movement.
	if( dx == 0. && dy == 0. )
		return false;
	// call appropriate methods
	unsigned int modKeyMask = _ga_t1->getModKeyMask();
	unsigned int buttonMask = _ga_t1->getButtonMask();
	if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	else if( ( buttonMask == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON ) ||
			( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && modKeyMask & osgGA::GUIEventAdapter::MODKEY_CTRL ) ||
			( buttonMask == (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ) )
		return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
	else if( buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
		return performMovementRightMouseButton( eventTimeDelta, dx, dy );
	else
		return performMovementLeftMouseButton( eventTimeDelta, dx, dy );
	return false;
}

bool OVNIController::handleMouseMove(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	addMouseEvent( ea );
	if ( performMovement() )
		aa.requestRedraw();
	aa.requestContinuousUpdate( false );
	_thrown = false;
	if( !mouseFree )
	{
		// Warp horizontal
		if( _ga_t0->getX() > windowW*0.9 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( windowW*0.2, _ga_t0->getY() );
		}
		else
		if( _ga_t0->getX() < windowW*0.1 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( windowW*0.8, _ga_t0->getY() );
		}
		// Warp vertical
		if( _ga_t0->getY() > windowH*0.9 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( _ga_t0->getX(), windowH*0.2 );
		}
		else
		if( _ga_t0->getY() < windowH*0.1 )
		{
			Warping = true;
			mouseFree = true;
			aa.requestWarpPointer( _ga_t0->getX(), windowH*0.8 );
		}
	}
	return true;
}

void calcAcc(osgViewer::Viewer* viewer) // calcula aceleração do OVNI
{
	// velocidade frontal
	if (!keyW && !keyS)
	{
		velZ *= 0.9;
		if( abs(velZ) < 0.01 )
			velZ = 0;
	}
	else
	{
		if (keyW && velZ > -maxVel)
			velZ -= acc;
		if (keyS && velZ < maxVel)
			velZ += acc;
	}
	// velocidade lateral
	if (!keyA && !keyD)
	{
		velX *= 0.9;
		if( abs(velX) < 0.01 )
			velX = 0;
	}
	else
	{
		if (keyA && velX > -maxVel)
			velX -= acc;
		if (keyD && velX < maxVel)
			velX += acc;
	}
	// velocidade vertical
	if (!keyR && !keyF)
	{
		velY *= 0.9;
		if( abs(velY) < 0.01 )
			velY = 0;
	}
	else
	{
		if (keyR && velY < maxVel)
			velY += acc;
		if (keyF && velY > -maxVel)
			velY -= acc;
	}
	camSpeed.set(velX, velY, velZ);

	// gravidade
	osg::Vec3d vel, pos;
	osg::Matrix matrix;
	osg::Quat rotation;
	osg::ref_ptr<osg::MatrixTransform> bloco;
	for (int i=0; i<blocos->getNumChildren(); i++)
	{
		bloco = dynamic_cast<osg::MatrixTransform*>( blocos->getChild(i) );
		bloco->getChild(0)->getUserValue( "speed", vel );
		if ( vel != osg::Vec3d( 0,0,0 ) )
		{
			vel += osg::Vec3d( 0,0,-0.005 ); // acelera
			matrix = bloco->getMatrix();
			rotation = matrix.getRotate();
			pos = matrix.getTrans();
			pos += rotation*vel;
			float chao = controller->pickDown( pos ) + 0.05; // 0.5 é relativo ao tamanho do cubo
			if ( pos.z() <= chao ) // para no chão
			{
				vel = osg::Vec3d( 0,0,0 );
				pos.z() = chao;
				bloco->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
			}
			else // continua caindo
			{
				bloco->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) * osg::Matrix::translate( pos ) );
			}
			bloco->getChild(0)->setUserValue( "speed", vel );
		}
	}
	// Sol
	bool solMudou = false;
	if (keyL)
	{
		solMudou = true;
		solLO += velSol;
		if( solLO > pi)
			solLO -= 2*pi;
	}
	else
	if (keyO)
	{
		solMudou = true;
		solLO -= velSol;
		if( solLO < pi)
			solLO += 2*pi;
	}
	if (keyI)
	{
		solMudou = true;
		solNS += velSol;
		if( solNS > pi)
			solNS -= 2*pi;
	}
	else
	if (keyK)
	{
		solMudou = true;
		solNS -= velSol;
		if( solNS < pi)
			solNS += 2*pi;
	}
	if (solMudou)
	{
		posSol = osg::Vec3d( lonC+rCeu*sin(solLO), latC+rCeu*sin(solNS), rCeu*cos(solLO)*cos(solNS) );
		osg::MatrixTransform* SolM = dynamic_cast<osg::MatrixTransform*>( Sol.get() );
		SolM->setMatrix( osg::Matrix::translate( posSol ) );
	}
}

osg::PositionAttitudeTransform* criaBotao( const char* fName, const char* name, int n )
{
	osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( fName );
	if (!image)
		return nullptr;
	osg::Geometry* polyGeom = new osg::Geometry();
	int bussP = 256; // porque compass.png é 512x512 pixels
	osg::Vec3 myCoords[] =
	{
		osg::Vec3(-bussP,bussP,0),
		osg::Vec3(-bussP,-bussP,0),
		osg::Vec3(bussP,-bussP,0),
		osg::Vec3(bussP,bussP,0)
	};
	polyGeom->setVertexArray(new osg::Vec3Array(4,myCoords));
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); // 0.5 no final deixa translúcido
	polyGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	polyGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
	osg::Vec2 myTexCoords[] =
	{
		osg::Vec2(0,1),
		osg::Vec2(0,0),
		osg::Vec2(1,0),
		osg::Vec2(1,1)
	};
	polyGeom->setTexCoordArray(0,new osg::Vec2Array(4,myTexCoords));
	unsigned short myIndices[] = { 0, 1, 3, 2 };
	polyGeom->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLE_STRIP,4,myIndices));
	osg::StateSet* stateset = new osg::StateSet;
	osg::Texture2D* texture = new osg::Texture2D;
	texture->setImage(image);
	stateset->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);
	polyGeom->setStateSet(stateset);
	polyGeom->setName("GeometryBotao");
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(polyGeom);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);
	geode->setName(name);

	osg::ref_ptr<osg::PositionAttitudeTransform> botao = new osg::PositionAttitudeTransform();
	float escala = windowH/9000.0f;
	botao->setPosition(osg::Vec3d(windowW/2 + n*escala*600,-40,0));
	botao->setScale(osg::Vec3(escala,escala,escala));
	botao->addChild(geode);
	botao->setName("patBotao");
	return botao.release();
}

void criaMenus() // examples/osggeometry/osggeometry.cpp
{
	osg::Geometry* polyGeom = new osg::Geometry();
	int mnuW = windowW;
	int mnuH = 120;
	osg::Vec3 myCoords[] =
	{
		osg::Vec3(0,0,0),
		osg::Vec3(0,-mnuH,0),
		osg::Vec3(mnuW,-mnuH,0),
		osg::Vec3(mnuW,0,0)
	};
	polyGeom->setVertexArray(new osg::Vec3Array(4,myCoords));
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.0f,0.0f,0.0f,0.5f)); // 0.5 = translúcido
	polyGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	polyGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
	osg::Vec2 myTexCoords[] =
	{
		osg::Vec2(0,1),
		osg::Vec2(0,0),
		osg::Vec2(1,0),
		osg::Vec2(1,1)
	};
	polyGeom->setTexCoordArray(0,new osg::Vec2Array(4,myTexCoords));
	unsigned short myIndices[] = { 0, 1, 3, 2 };
	polyGeom->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLE_STRIP,4,myIndices));
	osg::StateSet* stateset = new osg::StateSet;
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON); // deixa fundo do PNG transparente
	stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
	polyGeom->setStateSet(stateset);
	polyGeom->setName("GeometryMenu");
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(polyGeom);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	geode->setName("GeodeMenu");
	menu = new osg::PositionAttitudeTransform();
	menu->setNodeMask(0); // menu começa oculto
	menu->setPosition(osg::Vec3d(0,windowH,-0.6f)); // com 1 no final não aparece na tela; com -1 no final não aparece no pick, com 0 no final o texto não aparece sobre o menu !!
	menu->addChild( criaBotao("icons/hidreletrica.png", "Hidrelétrica", 0) );
	menu->addChild( criaBotao("icons/indigena.png", "Terra Indígena", 1) );
	menu->addChild(geode);
	menu->setName("patMenu");
}

void criaBussola() // examples/osggeometry/osggeometry.cpp
{
	osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile("compass.png");
	if (!image)
		return;
	osg::Geometry* polyGeom = new osg::Geometry();
	int bussP = 64; // porque compass.png é 128x128 pixels
	osg::Vec3 myCoords[] =
	{
		osg::Vec3(-bussP,bussP,0),
		osg::Vec3(-bussP,-bussP,0),
		osg::Vec3(bussP,-bussP,0),
		osg::Vec3(bussP,bussP,0)
	};
	polyGeom->setVertexArray(new osg::Vec3Array(4,myCoords));
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); // 0.5 no final deixa translúcido
	polyGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	polyGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
	osg::Vec2 myTexCoords[] =
	{
		osg::Vec2(0,1),
		osg::Vec2(0,0),
		osg::Vec2(1,0),
		osg::Vec2(1,1)
	};
	polyGeom->setTexCoordArray(0,new osg::Vec2Array(4,myTexCoords));
	unsigned short myIndices[] = { 0, 1, 3, 2 };
	polyGeom->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLE_STRIP,4,myIndices));
	osg::StateSet* stateset = new osg::StateSet;
	osg::Texture2D* texture = new osg::Texture2D;
	texture->setImage(image);
	stateset->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON); // deixa fundo do PNG transparente
	polyGeom->setStateSet(stateset);
	polyGeom->setName("GeometryBussola");
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(polyGeom);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	geode->setName("GeodeBussola");
	bussola = new osg::PositionAttitudeTransform();
	bussola->setPosition(osg::Vec3d(windowW-bussP,bussP,0));
	bussola->addChild(geode);
}

osg::Node* createLightSource( unsigned int num, const osg::Vec3& trans, const osg::Vec4& color )
{
	osg::ref_ptr<osg::Light> light = new osg::Light;
	light->setLightNum( num );
	light->setDiffuse( color );
	light->setPosition( osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
	lightSource->setLight( light );
	osg::ref_ptr<osg::MatrixTransform> sourceTrans = new osg::MatrixTransform;
	sourceTrans->setMatrix( osg::Matrix::translate(trans) );
	sourceTrans->addChild( lightSource.get() );
	return sourceTrans.release();
}

osg::Node* createLabel(const osg::Vec3& pos, float size, const std::string& label, osgText::Text::AxisAlignment axisAlignment)
{
	osg::Geode* geode = new osg::Geode();
	geode->setName("Geode-Nome da cidade");
	{
		osgText::Text* text = new osgText::Text;
		geode->addDrawable( text );
		text->setPosition(pos);
		text->setName("Text-Nome da cidade");
		text->setCharacterSize(size);
		text->setAxisAlignment(axisAlignment);
		text->setAlignment(osgText::Text::CENTER_CENTER);
		text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_RIGHT);
		text->setBackdropColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.5f));
		text->setFont( g_font.get() ); // Charter.ttf
		text->setText( label, osgText::String::ENCODING_UTF8 );
	}
	return geode;
}

void criaVoronoi(int n)
{
	std::vector<Vector2> points;
	// Generate points
	for( int i=0; i<n; i++ )
	{
		points.push_back( Vector2{ lonMin + (lonMax-lonMin)*(double)rand()/(double)RAND_MAX ,
									latMin + (latMax-latMin)*(double)rand()/(double)RAND_MAX } );
	}
	// Construct diagram
	FortuneAlgorithm algorithm(points);
	algorithm.construct();
	// Bound the diagram
	algorithm.bound(Box{lonMin-1, latMin-1, lonMax+1, latMax+1}); // Take the bounding box slightly bigger than the intersection box
	VoronoiDiagram diagram = algorithm.getDiagram();
	// Intersect the diagram with a box
	bool valid = diagram.intersect(Box{lonMin, latMin, lonMax, latMax});
	if (!valid)
		throw std::runtime_error("An error occured in the box intersection algorithm");
	// mostra o resultado
	osg::ref_ptr<osg::Geode> shpGeode;
	osg::Geometry* linesGeom;
	osg::Vec3Array* vertices;
	grpVoronoi = new osg::Group;
	grpVoronoi->setName("grpVoronoi");
	grpCidades = new osg::Group;
	grpCidades->setName("grpCidades");
	for (std::size_t i = 0; i < diagram.getNbSites(); ++i)
	{
		const VoronoiDiagram::Site* site = diagram.getSite(i);
		Vector2 center = site->point;
		VoronoiDiagram::Face* face = site->face;
		VoronoiDiagram::HalfEdge* halfEdge = face->outerComponent;
		if (halfEdge == nullptr)
			continue;
		VoronoiDiagram::HalfEdge* start = halfEdge;
		int lados = 0;
		float minX=lonMax, maxX=lonMin, minY=latMax, maxY=latMin; // Bounding Box deste site
		while (halfEdge != nullptr)
		{
			lados++;
			if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
			{
				Vector2 origin = halfEdge->origin->point;
				if( origin.x < minX )
					minX = origin.x;
				if( origin.x > maxX )
					maxX = origin.x;
				if( origin.y < minY )
					minY = origin.y;
				if( origin.y > maxY )
					maxY = origin.y;
			}
			halfEdge = halfEdge->next;
			if (halfEdge == start)
				break;
		}
		tCidades::iterator iCidade;
		int maiorPop = 0;
		std::string maiorNome;
		float maiorLon, maiorLat;
		for( iCidade=Cidades.begin(); iCidade!=Cidades.end(); ++iCidade )
		{
			if( iCidade->lon>=minX && iCidade->lon<=maxX && iCidade->lat>=minY && iCidade->lat<=maxY ) // cidade está dentro do bounding box
			{
				int i;
				int j;
				bool result = false;
				while (halfEdge != nullptr)
				{
					if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
					{
						Vector2 origin = halfEdge->origin->point; // j
						Vector2 dest = halfEdge->destination->point; // i
						if ((dest.y > iCidade->lat) != (origin.y > iCidade->lat) &&
							(iCidade->lon < (origin.x - dest.x) * (iCidade->lat - dest.y) / (origin.y-dest.y) + dest.x)) {
							result = !result;
						}
					}
					halfEdge = halfEdge->next;
					if (halfEdge == start)
						break;
				}
				if (result) // cidade está no polígono
				{
					if( iCidade->pop > maiorPop )
					{
						maiorPop = iCidade->pop;
						maiorNome = iCidade->nome + "/" + iCidade->UF;
						maiorLon = iCidade->lon;
						maiorLat = iCidade->lat;
					}
				}
			}
		}
		// escreve o nome da cidade no mapa
		if( maiorPop > 0 )
		{
			osg::AutoTransform* at = new osg::AutoTransform;
			at->setPosition( osg::Vec3(maiorLon,maiorLat,controller->pickDown( osg::Vec3d(maiorLon,maiorLat,1) )+0.2) );
			at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
			at->setAutoScaleToScreen(true);
			at->setMinimumScale(1); // ?? Não entendi como isso funciona
			at->setMaximumScale(2);
			at->addChild(createLabel(osg::Vec3(0,0,0),0.25,maiorNome,osgText::Text::XY_PLANE));
			at->setName("AT-Nome da cidade");
			grpCidades->addChild(at);
		}
		while (halfEdge != nullptr)
		{
			if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
			{
				Vector2 origin = halfEdge->origin->point;
				Vector2 dest = halfEdge->destination->point;
				
				// Linhas sobre o mapa
				shpGeode = new osg::Geode();
				linesGeom = new osg::Geometry();
				vertices = new osg::Vec3Array;
				vertices->push_back(osg::Vec3d(origin.x,origin.y,1));
				vertices->push_back(osg::Vec3d(dest.x,dest.y,1));
				linesGeom->setVertexArray(vertices);
				osg::Vec4Array* colors = new osg::Vec4Array;
				colors->push_back(osg::Vec4(0,0,0,1));
				linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
				osg::Vec3Array* normals = new osg::Vec3Array;
				normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
				linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
				linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,2));
				shpGeode->addDrawable(linesGeom);
				shpGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
				grpVoronoi->addChild( shpGeode.get() );
			}
			halfEdge = halfEdge->next;
			if (halfEdge == start)
				break;
		}
		grpVoronoi->setNodeMask(0); // começa invisível
	}
}

// https://stackoverflow.com/a/868894/1086511
class InputParser
{
public:
	InputParser (int &argc, char **argv)
	{
		for (int i=1; i < argc; ++i)
			this->tokens.push_back(std::string(argv[i]));
	}
	const std::string& getCmdOption(const std::string &option) const
	{
		std::vector<std::string>::const_iterator itr;
		itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end())
			return *itr;
		static const std::string empty_string("");
		return empty_string;
	}
	bool cmdOptionExists(const std::string &option) const
	{
		return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
	}
private:
	std::vector <std::string> tokens;
};

int main( int argc, char** argv )
{
	read_png_file("br.png");
	// cria arquivo de log
	mkdir("log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // cria a pasta log, se não existir
	time_t t = time(nullptr);
	struct tm t_tm = *std::localtime(&t);
	std::ostringstream t_ss;
	t_ss << std::put_time(&t_tm, "%Y%m%d_%H%M%S");
	std::string t_str = t_ss.str();
	logFileName = "log/log_"+t_str+".txt";
	// começa o programa
	log("Começou");
	
	InputParser input(argc, argv);
	if( input.cmdOptionExists("-h") )
	{
		std::cout << "SimNation beta, um simulador nacional de políticas socioambientais e socioeconômicas.\n";
		std::cout << "Como usar: sn [opções...]\n\n";
		std::cout << "Opções:\n";
		std::cout << "	-h		Mostra esta mensagem\n";
		std::cout << "	-n N		N = Número de células de voronoi [2..1000, padrão 200]\n";
		std::cout << "			(quanto mais células, mais cidades serão mostradas)\n";
		std::cout << "	-v N		N = velocidade do OVNI [1..50, padrão 10]\n";
		return 1;
	}
	if( input.cmdOptionExists("-n") )
	{
		nVoronoi = atoi(input.getCmdOption("-n").c_str());
		if( nVoronoi < 2)
			nVoronoi = 2;
		else
		if( nVoronoi > 1000)
			nVoronoi = 1000;
	}
	if( input.cmdOptionExists("-v") )
	{
		ovniSpeed = atoi(input.getCmdOption("-v").c_str());
		if( ovniSpeed < 1)
			ovniSpeed = 1;
		else
		if( ovniSpeed > 50)
			ovniSpeed = 50;
	}
	
	std::srand(std::time(0));
	getCidades();
	blocos = new osg::Group;
	blocos->setName("grpBlocos");
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->setName("grpRoot");
	osg::ref_ptr<osg::Node> BRnode = osgDB::readNodeFile("BR.osgb");
	BRnode->setName("Terreno");
	root->addChild( BRnode );
	root->addChild( blocos );
	// Luz
	Sol = createLightSource( 0, posSol, osg::Vec4(0.8f,0.8f,0.8f,1.0f) );
	root->getOrCreateStateSet()->setMode( GL_LIGHT0, osg::StateAttribute::ON );
	root->addChild( Sol );
	// Viewer
	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	controller = new OVNIController(&viewer);
	viewer.setCameraManipulator(controller);
	if( argc > 1 && strcmp( argv[1], "j" ) == 0 )
		viewer.setUpViewInWindow(0, 0, 800, 600); // deprecated? osgViewer::View::apply( ViewConfig* config )
	viewer.realize();
	osgViewer::ViewerBase::Cameras cameras;
	viewer.getCameras(cameras);
	osg::Camera* camera = cameras[0];
	camera->setName("cameraMain");
	// Posição inicial da câmera
	osg::Quat quad0;
	controller->setTransformation(osg::Vec3(lonC,latC,rCeu), quad0 ); // camLon (-74-34.8)/2 = -54.4, camLat (5.3333333-33.8666667)/2 = -14.2666667, camAlt = 80
	// Detecta coordenadas da janela
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	window = windows.begin();
	(*window)->useCursor(false);
	(*window)->getWindowRectangle(windowX, windowY, windowW, windowH);
	std::cout << "Tamanho da janela: " << windowW << "x" << windowH << "\n";
	// Linhas do HUD (examples/osggeometry/osggeometry.cpp)
	crossGeode = new osg::Geode();
	osg::Geometry* linesGeom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array(4);
	int crossP = 20;
	(*vertices)[0].set(windowW/2 - crossP, windowH/2, 0);
	(*vertices)[1].set(windowW/2 + crossP, windowH/2, 0);
	(*vertices)[2].set(windowW/2, windowH/2 - crossP, 0);
	(*vertices)[3].set(windowW/2, windowH/2 + crossP, 0);
	linesGeom->setVertexArray(vertices);
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
	linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);
	linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,4));
	crossGeode->addDrawable(linesGeom);
	crossGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED); // With lighting off, geometry color ignores the viewing angle
	// Texto do HUD
	text = createText(osg::Vec3(10, windowH-30, -0.5f), "", windowW/64); // com 0 no final o menu tampa, com 0.1 ou 1 no final não aparece na tela, com -1 no final aparece atrás do menu
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	textGeode->addDrawable( text );
	osg::ref_ptr<osg::Camera> hudCamera = createHUDCamera(0, windowW, 0, windowH); // cria um HUD do tamanho da janela, não mais 800x600 (piorou o desempenho?)
	hudCamera->addChild( crossGeode.get() );
	criaBussola();
	criaMenus();
	hudCamera->addChild( bussola );
	hudCamera->addChild( menu );
	hudCamera->addChild( textGeode.get() );
	root->addChild( hudCamera.get() );
	// voronoi para selecionar maiores cidades de cada região
	criaVoronoi(nVoronoi);
	root->addChild( grpVoronoi );
	root->addChild( grpCidades );

	// insere vaca
	/*osg::ref_ptr<osg::Node> vaca = osgDB::readRefNodeFile("cow.osg"); // vem com o Images/reflect.rgb
	vaca->setName("nodeVaca");
	osg::ref_ptr<osg::MatrixTransform> trVaca = new osg::MatrixTransform;
	trVaca->setMatrix( osg::Matrix::scale( 0.1, 0.1, 0.1 ) *
						osg::Matrix::translate( osg::Vec3(lonC, latC, controller->pickDown( osg::Vec3(lonC,latC,1) )+0.3 ) ) );
	trVaca->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
	trVaca->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	trVaca->setName("trVaca");
	trVaca->addChild( vaca );
	root->addChild(trVaca);*/

	grpHidreletrica = new osg::Group;
	grpHidreletrica->setName("grpHidreletrica");
	grpHidreletrica->setNodeMask(0); // começa invisível

	grpIndigena = new osg::Group;
	grpIndigena->setName("grpIndígena");
	grpIndigena->setNodeMask(0); // começa invisível

	getLocais();
	
	std::cout << "Nº hidrelétricas: " << std::to_string( grpHidreletrica->getNumChildren() ) << "\n";
	std::cout << "Nº terras indígenas: " << std::to_string( grpIndigena->getNumChildren() ) << "\n";
	
	root->addChild( grpHidreletrica );
	root->addChild( grpIndigena );
	
	// cria cubo com textura
	
	/*osg::ref_ptr<osg::Geometry> cubo = criaCubo();
	osg::ref_ptr<osg::MatrixTransform> mtCubo = new osg::MatrixTransform;
	mtCubo->setMatrix( osg::Matrix::scale( 0.5, 0.5, 0.5 ) *
		osg::Matrix::translate( osg::Vec3(lonC, latC, 1) ) );
	mtCubo->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
	mtCubo->setName("mtCubo");
	mtCubo->addChild( cubo );
	root->addChild( mtCubo );*/

	while ( !viewer.done() )
	{
		calcAcc( &viewer );
		if (Homing) // volta a câmera à posição inicial ao apertar Home
		{
			controller->setTransformation(osg::Vec3(lonC,latC,rCeu), quad0 );
			Homing = false;
		}
		viewer.frame();
	}
	log("Terminou");
	return 0;
}

// g++ terreno1.cpp -lpng16 -losg -losgDB -losgGA -losgText -losgUtil -losgViewer -o terreno1
