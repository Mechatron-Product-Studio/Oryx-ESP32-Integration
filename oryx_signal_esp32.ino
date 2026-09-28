/**
 * Oryx Signal Studio — generador de señal sobre ESP32
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * El micro recibe la señal que está abierta en el editor y la reproduce por
 * sus salidas. No se conecta a ninguna nube y a ningún servidor: todo lo que
 * necesita para sincronizarse con la app está acá adentro.
 *
 * ── Dos formas de llegar ───────────────────────────────────────────────────
 * USB   → el mismo cable con el que se carga este sketch. Es lo más directo:
 *         no hay red que cambiar ni dirección que averiguar. La app habla por
 *         el puerto serie con un protocolo de texto, una orden por línea.
 * WiFi  → el equipo levanta siempre su propia red y, si tiene una guardada,
 *         además se une a la de casa. Estando en la red de casa la
 *         computadora no cambia de red y conserva internet. Por WiFi se habla
 *         exactamente el mismo protocolo, por un WebSocket.
 *
 * Cada equipo tiene nombre propio (Oryx-Signal-XXXX, con parte de su MAC) y
 * conoce a los demás de la red por mDNS: la app los lista a todos sin que haya
 * que escribir ninguna dirección, y se puede pasar de uno a otro.
 *
 * Las credenciales de la red de casa se cargan una sola vez desde el portal
 * del equipo, quedan guardadas en memoria y no hace falta recompilar para
 * cambiarlas. El portal es cautivo: al unirse a la red propia, el teléfono o
 * la computadora lo abren solos. Si no, está en http://192.168.4.1.
 *
 * ── Cómo llega la señal ────────────────────────────────────────────────────
 * La app arma la señal que está en pantalla y la manda entera antes de
 * arrancar. Nunca se transmite mientras se reproduce: el micro reproduce de su
 * propia memoria, así que ni el WiFi ni el cable afectan al tiempo de la
 * señal.
 *
 *   digital   → una fila por cada cambio de estado: en qué paso ocurre y qué
 *               valor toma cada canal, todos juntos en una máscara de bits.
 *   analógico → la app muestrea la curva y manda los niveles ya listos. Es la
 *               forma más fiel: lo que sale por el DAC es exactamente lo que
 *               se dibujó, con sus curvas y sus esquinas redondeadas.
 *
 * ── Los dos núcleos ────────────────────────────────────────────────────────
 * La reproducción vive sola en el núcleo 1 y no se detiene por nada: es un
 * bucle de espera fina que no puede ceder tiempo sin perder precisión. La red
 * y el puerto serie corren en el núcleo 0, junto con la pila WiFi, y sí ceden
 * en cada vuelta. Por eso el vigilante del núcleo 1 se apaga a propósito en
 * setup(): el bucle de espera es justamente lo que ese vigilante castigaría.
 *
 * ── Voltaje ────────────────────────────────────────────────────────────────
 * En analógico el voltaje escala directo lo que sale por el DAC (0 – 3,3 V).
 * En digital una salida GPIO no sabe de niveles intermedios, así que el valor
 * se saca por PIN_REFERENCIA como tensión de referencia continua: es la que
 * alimenta la etapa externa (divisor, buffer o traductor de nivel) que fija la
 * altura real de los pulsos. Sin esa etapa, los pines salen siempre a 3,3 V.
 *
 * Placa: ESP32 clásico (WROOM / DevKit v1). Sin librerías externas: todo lo
 * que se incluye viene con el core de Arduino para ESP32 (2.x o 3.x).
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <mdns.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <stdarg.h>

// ═══════════════════════════════════════════════════════════════════════════
// AJUSTES
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Comienzo del nombre de cada equipo. Se completa al arrancar con cuatro
 * caracteres de su MAC (Oryx-Signal-A1B2): así varios equipos pueden estar
 * juntos y la app los lista por separado. Ese mismo nombre es el de la red
 * propia y el que se ve en la app.
 */
static const char* PREFIJO_NOMBRE = "Oryx-Signal";

/** Clave de la red propia, que existe siempre: es el portal y el último recurso. */
static const char* AP_CLAVE = "oryx1234";          // mínimo 8 caracteres

/**
 * Nombre común en la red de casa: oryx-signal.local
 *
 * Lo comparten todos los equipos a propósito. La app solo lo usa como puerta
 * de entrada: cualquier equipo que conteste le sirve, porque cada uno conoce a
 * los demás (ver VECINOS) y le pasa la lista entera.
 */
static const char* MDNS_NOMBRE = "oryx-signal";

/** Servicio mDNS con el que se encuentran los equipos entre sí. */
static const char* MDNS_SERVICIO = "_oryx";
static const char* MDNS_PROTO    = "_tcp";

/** Nombre propio de este equipo y su parte única. Se arman en setup(). */
static char nombreEquipo[24];
static char idEquipo[8];

/** Salidas digitales, en el mismo orden que los canales de la app. */
static const uint8_t PINES[] = { 16, 17, 18, 19, 21, 22, 23, 13 };
static const uint8_t MAX_CANALES_D = sizeof(PINES) / sizeof(PINES[0]);

/** Salidas analógicas: los dos DAC del ESP32. */
static const uint8_t PINES_DAC[] = { 25, 26 };
static const uint8_t MAX_CANALES_A = sizeof(PINES_DAC) / sizeof(PINES_DAC[0]);

/** Tensión de referencia para la etapa externa del modo digital. */
static const uint8_t PIN_REFERENCIA = 25;

/** Velocidad del puerto serie. Tiene que coincidir con la de la app. */
static const uint32_t BAUDIOS = 115200;

/**
 * Buffer de entrada del puerto serie.
 *
 * Los 256 bytes de fábrica no alcanzan: una señal grande son decenas de miles
 * de caracteres seguidos, sin control de flujo que los frene, y el buffer
 * tiene que absorber lo que llegue mientras la tarea está en otra cosa.
 */
static const size_t RX_SERIE = 4096;

/** Tope de la señal que entra en memoria. */
#define MAX_FILAS     2048               // cambios de estado, modo digital
#define MAX_MUESTRAS  6000               // muestras por canal, modo analógico
#define RX_BUF        8192               // mensaje de WebSocket más largo admitido

/** Tensión de fondo de escala del DAC, en milivoltios. */
static const uint32_t V_MAX_MV = 3300;

/** Tope de la velocidad. Con 0 rpm manda el tiempo de paso. */
static const uint32_t RPM_MAX = 8000;

/** Ciclos de la señal: cuántas vueltas del cigüeñal trae una pasada (CKP/CMP). */
static const uint32_t CICLOS_MIN = 1;
static const uint32_t CICLOS_MAX = 8;

/** Versión del protocolo. La app se niega a hablar con otra. */
static const uint8_t PROTO = 1;

/** Sin órdenes por este rato, del otro lado del cable ya no hay nadie. */
static const uint32_t SILENCIO_SERIE_MS = 10000;

/** Cuánto se espera a entrar a la red de casa antes de darla por perdida. */
static const uint32_t PLAZO_RED_MS = 12000;

/** Cada cuánto se vuelve a probar la red de casa cuando no se pudo entrar. */
static const uint32_t REINTENTO_RED_MS = 60000;

/** Otros equipos que se recuerdan a la vez. */
static const uint8_t  MAX_VECINOS = 16;

/** Cada cuánto se pregunta en la red de casa quién más anda por ahí. */
static const uint32_t BUSQUEDA_CADA_MS = 2500;
static const uint32_t BUSQUEDA_PLAZO_MS = 1500;

/** Un equipo que no contesta por este rato se da por apagado. */
static const uint32_t VECINO_VIGENCIA_MS = 9000;

/**
 * Solo se busca mientras alguien mira la lista. Pasado este rato sin que la
 * pidan, el equipo deja de preguntar y no llena la red de consultas.
 */
static const uint32_t MIRADA_MS = 15000;

// ═══════════════════════════════════════════════════════════════════════════
// TIPOS DEL WEBSOCKET
// ═══════════════════════════════════════════════════════════════════════════
//
// Van acá arriba y no junto a su código por una costumbre del IDE: antes de
// compilar un .ino escribe la declaración de cada función por encima de la
// primera de ellas. Una función que recibe o devuelve uno de estos tipos
// quedaría declarada antes que el tipo, y no compilaría.

/** Estado de un SHA-1 a medio calcular. */
struct Sha1 {
  uint32_t h[5];
  uint8_t  buf[64];
  uint8_t  idx;
  uint64_t bits;
};

/** Qué salió de leer una trama. */
enum WsRead : uint8_t {
  WS_NADA,     // no había un mensaje completo todavía
  WS_TEXTO,    // llegó entero: está en el buffer
  WS_CERRAR,   // el cliente pidió cerrar, o la conexión se rompió
};

// ═══════════════════════════════════════════════════════════════════════════
// SEÑAL EN MEMORIA
// ═══════════════════════════════════════════════════════════════════════════

enum Modo : uint8_t { MODO_NADA = 0, MODO_DIGITAL = 1, MODO_ANALOGICO = 2 };

/** Un cambio de estado: en qué paso ocurre y cómo quedan todos los canales. */
struct Fila {
  float    pos;                          // paso, contando desde 0
  uint16_t mask;                         // bit i = canal i en alto
};

static Fila     filas[MAX_FILAS];
/** Cuántos pasos se sostiene cada fila hasta la siguiente (la última, hasta la vuelta). */
static float    durFila[MAX_FILAS];
static uint16_t nFilas        = 0;
static float    pasosTotales  = 0;       // largo de una vuelta completa

static uint16_t muestras[MAX_CANALES_A][MAX_MUESTRAS];
static uint32_t nMuestras     = 0;       // por canal
static uint8_t  spp           = 16;      // muestras por paso
static uint32_t nivelMax      = 4095;    // fondo de escala de las muestras

static uint8_t  nCanales      = 0;       // canales realmente cargados
static uint8_t  modo          = MODO_NADA;

// ═══════════════════════════════════════════════════════════════════════════
// ESTADO COMPARTIDO ENTRE LOS DOS NÚCLEOS
// ═══════════════════════════════════════════════════════════════════════════

static volatile bool     corriendo      = false;
static volatile bool     enMarcha       = false;   // la reproducción está adentro de un ciclo
static volatile bool     pedidoReinicio = false;

static volatile uint32_t mv       = 3300;          // amplitud pedida
static volatile uint32_t usPedido = 1000;          // microsegundos por paso
static volatile uint32_t rpm      = 0;             // 0 = manda usPedido
/**
 * Ciclos que trae la señal. Es un dato de la señal, igual que en el equipo de
 * SD: se guarda y se informa, pero no entra en el tiempo de paso (la fórmula
 * de las rpm no lo usa).
 */
static volatile uint32_t ciclosSenal = 2;
/**
 * El tiempo de paso que se usa de verdad. Va con decimales: redondearlo al
 * microsegundo corría la señal hasta un 1 % a rpm altas, porque el error se
 * repite en cada paso de la vuelta. Un float se escribe de una sola vez, así
 * que el otro núcleo nunca lo lee a medias.
 */
static volatile float    usPaso   = 1000.0f;

static volatile uint32_t ciclos    = 0;
static volatile float    posActual = 0;

/** Factor de amplitud, en punto fijo: dac = muestra * factorQ16 >> 16. */
static volatile uint32_t factorQ16 = 0;

// ═══════════════════════════════════════════════════════════════════════════
// SALIDAS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Bits del registro de salida de cada máscara de canales: gpioDe[m] tiene en
 * alto el pin de cada canal que está en alto en m. Con eso una fila se
 * escribe en dos accesos al registro, todos los canales en el mismo instante,
 * en vez de un digitalWrite por pin.
 */
static uint32_t gpioDe[256];

static void armarTablaGpio() {
  for (uint16_t m = 0; m < 256; m++) {
    uint32_t bits = 0;
    for (uint8_t i = 0; i < MAX_CANALES_D; i++) {
      if (m & (1u << i)) bits |= (1UL << PINES[i]);   // todos los PINES son < 32
    }
    gpioDe[m] = bits;
  }
}

/** Pone los canales cargados como dice la máscara, todos juntos. */
static inline void escribirMask(uint16_t m) {
  const uint8_t cargados = (uint8_t)((1u << nCanales) - 1);
  REG_WRITE(GPIO_OUT_W1TS_REG, gpioDe[m & cargados]);
  REG_WRITE(GPIO_OUT_W1TC_REG, gpioDe[~m & cargados]);
}

/** Recalcula el factor de amplitud a partir del voltaje y la resolución. */
static void recalcularFactor() {
  const uint64_t techo = (uint64_t)nivelMax * V_MAX_MV;
  factorQ16 = techo == 0 ? 0 : (uint32_t)(((uint64_t)255 * mv * 65536ULL) / techo);
}

/** Nivel de la app llevado a los 8 bits del DAC, con el voltaje ya aplicado. */
static inline uint8_t aDac(uint16_t nivel) {
  const uint32_t v = ((uint32_t)nivel * factorQ16) >> 16;
  return v > 255 ? 255 : (uint8_t)v;
}

/** Referencia continua del modo digital. */
static void escribirReferencia() {
  const uint32_t v = ((uint32_t)255 * mv) / V_MAX_MV;
  dacWrite(PIN_REFERENCIA, v > 255 ? 255 : (uint8_t)v);
}

/** Todo abajo: es donde queda la señal cuando está parada. */
static void salidasIdle() {
  // Se barren TODOS los pines, no solo los canales cargados: puede quedar uno
  // en alto de una señal anterior con más canales que la de ahora.
  REG_WRITE(GPIO_OUT_W1TC_REG, gpioDe[0xFF]);
  if (modo == MODO_ANALOGICO) {
    for (uint8_t c = 0; c < MAX_CANALES_A; c++) dacWrite(PINES_DAC[c], 0);
  } else {
    escribirReferencia();
  }
}

/**
 * Resuelve el tiempo de paso: las rpm, si están puestas, mandan sobre él.
 *
 * Una vuelta de la señal es una vuelta del motor, así que 60 s / rpm se
 * reparten entre todos los pasos de la vuelta:
 *
 *   tiempo de paso (µs) = 60 000 000 / (rpm × pasos totales)
 *
 * Se guarda con decimales y sin tope: con un piso, la velocidad real dejaría
 * de ser la pedida. La reproducción lee el valor nuevo en el cambio siguiente,
 * sin cortar la vuelta en curso.
 */
static void resolverRitmo() {
  double us = (double)usPedido;
  if (rpm > 0 && pasosTotales > 0) {
    us = 60000000.0 / ((double)rpm * (double)pasosTotales);
  }
  if (us < 0.001) us = 0.001;
  usPaso = (float)us;
}

/** Prepara lo que la reproducción digital lee en cada fila. */
static void prepararFilas() {
  for (uint16_t i = 0; i < nFilas; i++) {
    float d = (i + 1 < nFilas)
      ? filas[i + 1].pos - filas[i].pos
      // La última se sostiene hasta el final de la vuelta y lo que falte hasta
      // la primera fila de la vuelta siguiente.
      : pasosTotales - filas[i].pos + filas[0].pos;
    durFila[i] = d > 0 ? d : 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// REPRODUCCIÓN — núcleo 1
// ═══════════════════════════════════════════════════════════════════════════
//
// Cómo se lleva el tiempo, que es lo que hace que la velocidad pedida sea la
// real: hay un instante ideal, con decimales de microsegundo, al que se le
// suma exactamente lo que dura cada fila. Solo se redondea el plazo absoluto
// al esperar, nunca lo que se suma, así que el redondeo no se acumula vuelta a
// vuelta. Y como cada fila lee usPaso en el momento, cambiar las rpm acelera o
// frena la señal en el acto, como un motor, sin volver al principio.

/**
 * Espera hasta el instante `t`, en microsegundos del reloj del micro.
 *
 * Devuelve false si mientras tanto pararon la señal o pidieron reiniciarla.
 * Cuando falta bastante cede el procesador; sobre el final gira en vacío, que
 * es la única forma de acertarle al microsegundo.
 */
static bool esperarHasta(int64_t t) {
  while (true) {
    if (!corriendo || pedidoReinicio) return false;
    const int64_t falta = t - (int64_t)esp_timer_get_time();
    if (falta <= 0) return true;
    if (falta > 2000) vTaskDelay(1);
  }
}

static void reproducirDigital() {
  double ideal = (double)esp_timer_get_time();
  uint16_t i = 0;

  while (corriendo) {
    escribirMask(filas[i].mask);
    posActual = filas[i].pos;

    ideal += (double)durFila[i] * (double)usPaso;
    if (!esperarHasta((int64_t)(ideal + 0.5))) return;

    if (++i >= nFilas) { i = 0; ciclos = ciclos + 1; }
  }
}

/**
 * La analógica no se recorre muestra por muestra sino por reloj: en cada
 * pasada se mira cuánto tiempo pasó, se avanza la posición en esa medida y se
 * escribe la muestra que toca. Si el DAC no llega a escribirlas todas (rpm
 * altas con muchas muestras por paso) se saltea alguna, pero la vuelta dura
 * siempre lo que tiene que durar.
 */
static void reproducirAnalogico() {
  int64_t  antes  = esp_timer_get_time();
  double   m      = 0;                    // posición, en muestras
  uint32_t ultima = UINT32_MAX;

  while (corriendo) {
    if (pedidoReinicio) return;

    // Una vuelta de nMuestras dura pasosTotales pasos: de ahí sale lo que dura
    // cada muestra, sin depender de que nMuestras sea justo pasos × spp.
    const double usMuestra = (double)usPaso * (double)pasosTotales / (double)nMuestras;
    const int64_t ahora = esp_timer_get_time();
    m += (double)(ahora - antes) / usMuestra;
    antes = ahora;
    while (m >= (double)nMuestras) { m -= (double)nMuestras; ciclos = ciclos + 1; }

    const uint32_t i = (uint32_t)m;
    if (i != ultima) {
      for (uint8_t c = 0; c < nCanales && c < MAX_CANALES_A; c++) {
        dacWrite(PINES_DAC[c], aDac(muestras[c][i]));
      }
      ultima = i;
      posActual = (float)i / (float)spp;
    }

    if (((double)(i + 1) - m) * usMuestra > 2000) vTaskDelay(1);
  }
}

static void tareaReproduccion(void*) {
  for (;;) {
    const bool hayQue = corriendo && modo != MODO_NADA &&
                        ((modo == MODO_DIGITAL && nFilas > 0) ||
                         (modo == MODO_ANALOGICO && nMuestras > 0));
    if (!hayQue) {
      if (enMarcha) { salidasIdle(); enMarcha = false; }
      vTaskDelay(5);
      continue;
    }
    if (pedidoReinicio) {
      pedidoReinicio = false;
      ciclos = 0;
      posActual = 0;
    }
    enMarcha = true;
    if (modo == MODO_DIGITAL) reproducirDigital();
    else                      reproducirAnalogico();
  }
}

/** Frena la reproducción y espera a que la otra tarea suelte los datos. */
static void detenerYEsperar() {
  corriendo = false;
  uint32_t t0 = millis();
  while (enMarcha && millis() - t0 < 500) delay(2);
}

// ═══════════════════════════════════════════════════════════════════════════
// WEBSOCKET — sin librerías externas
// ═══════════════════════════════════════════════════════════════════════════
//
// El navegador no sabe hablar TCP a secas, así que la única forma de que la
// página se conecte al micro es un WebSocket. Todo lo que hace falta para eso
// está acá y no depende de nada más que del core de Arduino:
//
//   · SHA-1 y Base64 escritos a mano, para firmar el apretón de manos sin
//     arrastrar mbedtls, que cambió de API entre las versiones 2 y 3 del core
//     y rompe la compilación según con cuál se arme el proyecto.
//   · Lectura y escritura de tramas de texto, que es lo único que usa el
//     protocolo de Oryx: todos los mensajes son líneas de texto.
//
// Lo que NO cubre, a propósito: compresión, extensiones, tramas binarias y
// varios clientes a la vez. El micro atiende a una sola página por vez.

// ── SHA-1 ──────────────────────────────────────────────────────────────────

static inline uint32_t sha1Rot(uint32_t v, uint8_t n) {
  return (v << n) | (v >> (32 - n));
}

static void sha1Block(Sha1& s) {
  uint32_t w[80];
  for (uint8_t i = 0; i < 16; i++) {
    w[i] = ((uint32_t)s.buf[i * 4] << 24) | ((uint32_t)s.buf[i * 4 + 1] << 16)
         | ((uint32_t)s.buf[i * 4 + 2] << 8) | (uint32_t)s.buf[i * 4 + 3];
  }
  for (uint8_t i = 16; i < 80; i++) {
    w[i] = sha1Rot(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
  }
  uint32_t a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3], e = s.h[4];
  for (uint8_t i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20)      { f = (b & c) | ((~b) & d);        k = 0x5A827999; }
    else if (i < 40) { f = b ^ c ^ d;                   k = 0x6ED9EBA1; }
    else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
    else             { f = b ^ c ^ d;                   k = 0xCA62C1D6; }
    uint32_t tmp = sha1Rot(a, 5) + f + e + k + w[i];
    e = d; d = c; c = sha1Rot(b, 30); b = a; a = tmp;
  }
  s.h[0] += a; s.h[1] += b; s.h[2] += c; s.h[3] += d; s.h[4] += e;
}

static void sha1Init(Sha1& s) {
  s.h[0] = 0x67452301; s.h[1] = 0xEFCDAB89; s.h[2] = 0x98BADCFE;
  s.h[3] = 0x10325476; s.h[4] = 0xC3D2E1F0;
  s.idx = 0; s.bits = 0;
}

static void sha1Update(Sha1& s, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    s.buf[s.idx++] = data[i];
    s.bits += 8;
    if (s.idx == 64) { sha1Block(s); s.idx = 0; }
  }
}

static void sha1Final(Sha1& s, uint8_t out[20]) {
  const uint64_t bits = s.bits;
  uint8_t pad = 0x80;
  sha1Update(s, &pad, 1);
  pad = 0x00;
  while (s.idx != 56) sha1Update(s, &pad, 1);
  // El largo en bits cierra el bloque, en big-endian, y no se cuenta a sí
  // mismo: por eso se escribe directo sobre el buffer y no con sha1Update.
  for (int8_t i = 7; i >= 0; i--) s.buf[s.idx++] = (uint8_t)(bits >> (i * 8));
  sha1Block(s);
  for (uint8_t i = 0; i < 5; i++) {
    out[i * 4]     = (uint8_t)(s.h[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(s.h[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(s.h[i] >> 8);
    out[i * 4 + 3] = (uint8_t)(s.h[i]);
  }
}

// ── Base64 ─────────────────────────────────────────────────────────────────

/** Codifica `len` bytes. `out` necesita 4*ceil(len/3)+1 lugares. */
static void base64Encode(const uint8_t* in, size_t len, char* out) {
  static const char TABLA[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t o = 0;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t bloque = (uint32_t)in[i] << 16;
    if (i + 1 < len) bloque |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < len) bloque |= (uint32_t)in[i + 2];
    out[o++] = TABLA[(bloque >> 18) & 63];
    out[o++] = TABLA[(bloque >> 12) & 63];
    out[o++] = (i + 1 < len) ? TABLA[(bloque >> 6) & 63] : '=';
    out[o++] = (i + 2 < len) ? TABLA[bloque & 63]        : '=';
  }
  out[o] = '\0';
}

// ── Tramas ─────────────────────────────────────────────────────────────────

/** Sufijo fijo que exige el protocolo antes de firmar la clave del cliente. */
static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/** Lee exactamente `n` bytes, o devuelve false si el cliente se calló. */
static bool wsReadN(WiFiClient& c, uint8_t* dst, size_t n, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  size_t leidos = 0;
  while (leidos < n) {
    if (c.available() > 0) {
      int r = c.read(dst + leidos, n - leidos);
      if (r > 0) { leidos += r; t0 = millis(); continue; }
    }
    if (!c.connected() && c.available() == 0) return false;
    if (millis() - t0 > timeoutMs) return false;
    delay(1);
  }
  return true;
}

/** Manda una trama de texto sin enmascarar (servidor → cliente). */
static bool wsSendText(WiFiClient& c, const char* texto, size_t len) {
  if (!c.connected()) return false;
  uint8_t cab[10];
  size_t n = 0;
  cab[n++] = 0x81;                       // FIN + opcode de texto
  if (len < 126) {
    cab[n++] = (uint8_t)len;
  } else if (len < 65536) {
    cab[n++] = 126;
    cab[n++] = (uint8_t)(len >> 8);
    cab[n++] = (uint8_t)len;
  } else {
    cab[n++] = 127;
    for (int8_t i = 7; i >= 0; i--) cab[n++] = (uint8_t)((uint64_t)len >> (i * 8));
  }
  if (c.write(cab, n) != n) return false;
  return c.write((const uint8_t*)texto, len) == len;
}

static bool wsSendText(WiFiClient& c, const char* texto) {
  return wsSendText(c, texto, strlen(texto));
}

/**
 * Lee una trama si hay una esperando.
 *
 * Las de control (ping · pong · close) se resuelven acá adentro: el que llama
 * solo se entera del texto. Un mensaje partido en varias tramas se va juntando
 * en `dst` hasta que llega la que trae el bit FIN, así que `largo` entra con lo
 * acumulado hasta ahora y sale actualizado.
 */
static WsRead wsReadText(WiFiClient& c, char* dst, size_t cap, size_t& largo) {
  if (!c.connected() && c.available() == 0) return WS_CERRAR;
  if (c.available() < 2) return WS_NADA;

  uint8_t cab[2];
  if (!wsReadN(c, cab, 2, 2000)) return WS_CERRAR;

  const bool    fin  = (cab[0] & 0x80) != 0;
  const uint8_t op   = cab[0] & 0x0F;
  const bool    mask = (cab[1] & 0x80) != 0;
  uint64_t      len  = cab[1] & 0x7F;

  if (len == 126) {
    uint8_t ext[2];
    if (!wsReadN(c, ext, 2, 2000)) return WS_CERRAR;
    len = ((uint64_t)ext[0] << 8) | ext[1];
  } else if (len == 127) {
    uint8_t ext[8];
    if (!wsReadN(c, ext, 8, 2000)) return WS_CERRAR;
    len = 0;
    for (uint8_t i = 0; i < 8; i++) len = (len << 8) | ext[i];
  }

  uint8_t clave[4] = { 0, 0, 0, 0 };
  if (mask && !wsReadN(c, clave, 4, 2000)) return WS_CERRAR;

  if (op == 0x8) return WS_CERRAR;       // close

  /* Un mensaje más grande que el buffer se lee igual y se tira: cortar la
     conexión a la mitad de una trama dejaría el stream sin sincronizar. */
  const bool cabe = (largo + (size_t)len) < cap;
  size_t escrito = largo;
  uint8_t trozo[64];
  uint64_t leidos = 0;
  while (leidos < len) {
    const size_t n = (size_t)min<uint64_t>(len - leidos, sizeof(trozo));
    if (!wsReadN(c, trozo, n, 2000)) return WS_CERRAR;
    if (cabe) {
      for (size_t i = 0; i < n; i++) {
        dst[escrito + i] = (char)(mask ? (trozo[i] ^ clave[(size_t)(leidos + i) & 3])
                                       : trozo[i]);
      }
      escrito += n;
    }
    leidos += n;
  }
  if (cabe) largo = escrito;

  if (op == 0x9) {                        // ping → pong sin cuerpo
    const uint8_t pong[2] = { 0x8A, 0x00 };
    c.write(pong, 2);
    largo = 0;
    return WS_NADA;
  }
  if (op == 0xA) { largo = 0; return WS_NADA; }   // pong suelto

  if (!fin) return WS_NADA;               // sigue en la trama que viene
  if (!cabe) { largo = 0; return WS_NADA; }
  dst[largo] = '\0';
  return WS_TEXTO;
}

/**
 * Contesta el apretón de manos.
 *
 * Devuelve false si lo que llegó no era una petición de WebSocket; en ese caso
 * ya quedó contestada con un 400 y el que llama tiene que cerrarla.
 */
static bool wsHandshake(WiFiClient& c) {
  char clave[64] = { 0 };
  bool esUpgrade = false;
  uint32_t t0 = millis();

  // Cabeceras hasta la línea en blanco. Solo interesan dos de ellas.
  while (millis() - t0 < 3000) {
    if (!c.connected() && c.available() == 0) return false;
    if (!c.available()) { delay(1); continue; }
    String linea = c.readStringUntil('\n');
    linea.trim();
    if (linea.length() == 0) break;
    t0 = millis();
    String baja = linea;
    baja.toLowerCase();
    if (baja.startsWith("upgrade:") && baja.indexOf("websocket") >= 0) esUpgrade = true;
    if (baja.startsWith("sec-websocket-key:")) {
      String v = linea.substring(linea.indexOf(':') + 1);
      v.trim();
      strncpy(clave, v.c_str(), sizeof(clave) - 1);
    }
  }

  if (!esUpgrade || clave[0] == '\0') {
    c.print(F("HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n"));
    return false;
  }

  Sha1 s;
  sha1Init(s);
  sha1Update(s, (const uint8_t*)clave, strlen(clave));
  sha1Update(s, (const uint8_t*)WS_GUID, strlen(WS_GUID));
  uint8_t hash[20];
  sha1Final(s, hash);
  char acepta[32];
  base64Encode(hash, 20, acepta);

  c.print(F("HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: "));
  c.print(acepta);
  c.print(F("\r\n\r\n"));
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// PÁGINAS — lo que se ve al entrar al equipo desde un navegador
// ═══════════════════════════════════════════════════════════════════════════
//
// Tienen la misma cara que Oryx Signal Studio: la cabecera oscura de la
// bienvenida con el logo y las dos trazas corriendo, el filo azul → violeta y
// una tarjeta con los mismos grises, radios y botones que los modales de la
// app. Siguen el tema del sistema (claro u oscuro), como la app cuando está en
// "seguir al sistema".
//
// Los colores son los de src/styles/colors.css pasados a hex, igual que en la
// plantilla de correo (supabase/email-templates): el equipo no puede leer las
// variables de la app, así que si la paleta cambia allá hay que traerla acá.
//
// ── Por qué existe el puente ────────────────────────────────────────────────
// Oryx Signal Studio vive en la web, o sea en https. Un navegador no deja que
// una página https abra un WebSocket ws:// contra una dirección de la red
// local: lo bloquea como "contenido mixto", y no hay bandera ni cabecera que
// lo habilite. Poner https en el ESP32 tampoco sirve: el certificado sería
// propio y el navegador lo rechazaría igual.
//
// Lo que sí está permitido es abrir una ventana nueva a una dirección http
// (eso es una navegación, no un recurso de la página), y que dos ventanas de
// orígenes distintos se hablen con postMessage. Así que el micro sirve él
// mismo la página del puente: corre en su propia dirección, desde donde el
// WebSocket es del mismo origen y no lo bloquea nadie, y le pasa los mensajes
// a la pestaña de la app.
//
//     app (https)  ⇄ postMessage ⇄  puente (http://equipo)  ⇄ ws ⇄  ESP32
//
// De paso, así también se esquiva el permiso de acceso a la red local que
// Chrome sumó en la versión 142: las navegaciones de nivel superior no están
// en alcance, y un pedido de local a local está excluido de forma explícita.
//
// El puente no interpreta nada: pasa texto para un lado y para el otro. Y solo
// habla con el origen que le llega en el hash de la URL, que lo pone la propia
// app al abrir la ventana.

/** Estilo compartido por todas las páginas del equipo. */
static const char ESTILO[] = R"CSS(<style>
:root{color-scheme:light dark;
 --fondo:#dfe8f8;--tarjeta:#ffffff;--borde:#bfc8d8;--caja:#f1fbff;--campo:#ffffff;
 --titulo:#0a0a0a;--texto:#6b7280;--rotulo:#9ca3af;
 --primario:#030213;--primario-texto:#ffffff;--enlace:#2563eb;
 --tonal:rgba(59,130,246,.10);--tonal-hover:rgba(59,130,246,.18);--tonal-borde:rgba(59,130,246,.40);--tonal-texto:#1d4ed8;
 --ok:#28a745;--error:#ef4444;--error-fondo:rgba(239,68,68,.08);--error-borde:rgba(239,68,68,.40);
 --cabecera:#17181b;--foco:rgba(59,130,246,.30)}
@media (prefers-color-scheme:dark){:root{
 --fondo:#17181b;--tarjeta:#26282b;--borde:#3e4043;--caja:#1f2123;--campo:#1f2123;
 --titulo:#ffffff;--texto:#a1a2a5;--rotulo:#717275;
 --primario:#fafafa;--primario-texto:#171717;--enlace:#60a5fa;
 --tonal:rgba(96,165,250,.12);--tonal-hover:rgba(96,165,250,.20);--tonal-borde:rgba(96,165,250,.40);--tonal-texto:#93c5fd;
 --error-fondo:rgba(239,68,68,.12);--cabecera:#111214}}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%}
body{margin:0;min-height:100vh;display:flex;justify-content:center;align-items:flex-start;padding:40px 16px;
 background:var(--fondo);color:var(--texto);font:16px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
.tarjeta{width:100%;max-width:440px;background:var(--tarjeta);border:1px solid var(--borde);border-radius:16px;overflow:hidden}
.cabecera{background-color:var(--cabecera);padding:22px 28px 22px;
 background-image:linear-gradient(rgba(255,255,255,.035) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.035) 1px,transparent 1px);
 background-size:16px 16px;background-position:center}
.cabecera.con-ondas{padding-bottom:6px}
.marca{display:flex;align-items:center;gap:12px;color:#fff;font-size:17px;line-height:22px;font-weight:600;letter-spacing:-.01em}
.marca svg{flex-shrink:0}
.ondas{display:block;width:100%;height:64px;margin-top:10px;
 -webkit-mask-image:linear-gradient(90deg,transparent,#000 16%,#000 84%,transparent);mask-image:linear-gradient(90deg,transparent,#000 16%,#000 84%,transparent)}
.ondas path{fill:none;stroke-linejoin:round;stroke-linecap:round;vector-effect:non-scaling-stroke}
.ondas .halo{stroke-width:7;opacity:.3}
.ondas .traza{stroke-width:2.25}
.corre{animation:correr 6s linear infinite}
.lenta{animation:correr-lento 10s linear infinite}
@keyframes correr{to{transform:translateX(120px)}}
@keyframes correr-lento{to{transform:translateX(160px)}}
.filo{height:3px;background:linear-gradient(90deg,#3b82f6,#7c3aed)}
.cuerpo{padding:28px}
.rotulo{margin:0;font-size:12px;line-height:16px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;color:var(--rotulo)}
h1{margin:8px 0 0;font-size:24px;line-height:30px;font-weight:600;letter-spacing:-.02em;color:var(--titulo)}
p{margin:8px 0 0}
b,strong{color:var(--titulo);font-weight:600}
a{color:var(--enlace);text-decoration:none}
a:hover{text-decoration:underline}
code{font:14px/1.4 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;color:var(--tonal-texto);word-break:break-all}
.caja{margin-top:20px;border:1px solid var(--borde);background:var(--caja);border-radius:16px;overflow:hidden}
.fila{display:flex;justify-content:space-between;align-items:center;gap:16px;padding:11px 16px;border-top:1px solid var(--borde);font-size:15px}
.fila:first-child{border-top:0}
.fila>span:first-child{flex-shrink:0}
.fila>span:last-child{color:var(--titulo);font-weight:500;text-align:right;min-width:0;overflow-wrap:anywhere}
.estado{display:inline-flex;align-items:center;gap:8px}
.punto{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--rotulo);flex-shrink:0}
.punto.ok{background:var(--ok)}
.punto.error{background:var(--error)}
.punto.espera{background:#60a5fa}
.pulso{animation:pulso 1.2s ease-in-out infinite}
@keyframes pulso{50%{opacity:.35}}
.encabezado{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-top:22px}
.etiqueta{display:block;font-size:14px;line-height:20px;font-weight:500;color:var(--titulo)}
label.etiqueta{margin-top:18px}
.campo{position:relative;margin-top:8px}
input{width:100%;height:44px;padding:0 14px;border:1px solid var(--borde);border-radius:12px;background:var(--campo);color:var(--titulo);
 font:inherit;font-size:16px;outline:0;transition:border-color .15s,box-shadow .15s}
input:focus{border-color:#3b82f6;box-shadow:0 0 0 3px var(--foco)}
.campo input{padding-right:84px}
.ver{position:absolute;right:6px;top:6px;height:32px;padding:0 10px;border:0;border-radius:8px;background:transparent;color:var(--texto);font:inherit;font-size:14px;cursor:pointer}
.ver:hover{background:var(--tonal);color:var(--tonal-texto)}
.redes{margin-top:8px;display:flex;flex-direction:column;gap:6px;max-height:252px;overflow-y:auto}
.red{display:flex;align-items:center;gap:12px;width:100%;min-height:44px;padding:10px 14px;border:1px solid var(--borde);border-radius:12px;
 background:var(--campo);color:var(--titulo);font:inherit;font-size:15px;text-align:left;cursor:pointer;transition:background .15s,border-color .15s}
.red:hover{background:var(--tonal);border-color:var(--tonal-borde)}
.red.sel{background:var(--tonal);border-color:var(--tonal-borde);color:var(--tonal-texto)}
.red span{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.red svg{width:16px;height:16px;flex-shrink:0;fill:currentColor}
.red .apagada{opacity:.25}
.red .candado{fill:none;stroke:currentColor;stroke-width:1.6;opacity:.7}
.vacio{display:flex;align-items:center;gap:10px;margin:0;padding:12px 14px;border:1px dashed var(--borde);border-radius:12px;font-size:15px}
.boton{display:flex;align-items:center;justify-content:center;gap:8px;width:100%;height:44px;margin-top:24px;padding:0 16px;border:1px solid transparent;border-radius:10px;
 background:var(--primario);color:var(--primario-texto);font:inherit;font-size:16px;font-weight:500;cursor:pointer;text-decoration:none;transition:box-shadow .15s,background .15s}
.boton:hover{box-shadow:0 0 0 2px var(--texto);text-decoration:none}
.boton:disabled{opacity:.5;cursor:default;box-shadow:none}
.boton.sec{background:transparent;color:var(--titulo);border-color:var(--borde);margin-top:10px}
.boton.sec:hover{background:var(--tonal);box-shadow:none}
.chico{height:32px;padding:0 10px;border:0;border-radius:8px;background:transparent;color:var(--enlace);font:inherit;font-size:14px;font-weight:500;cursor:pointer}
.chico:hover{background:var(--tonal)}
.aviso{margin:16px 0 0;padding:10px 14px;border:1px solid var(--error-borde);border-radius:12px;background:var(--error-fondo);color:var(--error);font-size:14px}
.pasos{margin:16px 0 0;padding-left:22px}
.pasos li{margin-top:8px}
.nota{margin-top:18px;font-size:14px;line-height:1.5;color:var(--rotulo)}
.separador{height:1px;background:var(--borde);margin:24px 0 0}
@media (max-width:480px){body{padding:16px}.cuerpo{padding:22px}.cabecera{padding-left:22px;padding-right:22px}h1{font-size:22px;line-height:28px}}
@media (max-height:460px){body{padding:12px}.cuerpo{padding:20px}.cabecera{padding-top:14px;padding-bottom:14px}}
@media (prefers-reduced-motion:reduce){.corre,.lenta,.pulso{animation:none}}
</style>)CSS";

/** El logo de la app, en su tinta clara: la cabecera de las páginas es oscura
 *  en los dos temas. Es el trazo de public/brand/oryx-logo-light.svg. */
static const char LOGO[] = R"SVG(<svg width="32" height="32" viewBox="0 0 32 32" aria-hidden="true"><g fill="none" stroke="#ededed" stroke-width="3.6" stroke-linecap="round" stroke-linejoin="round"><path d="M16 4.5L27.5 16L16 27.5L4.5 16Z"/><path d="M21.75 10.25L29.4 2.6M10.25 21.75L2.6 29.4"/></g></svg>)SVG";

/** El mismo logo como ícono de la pestaña, oscuro o claro según el tema. A 16 px
 *  usa un dibujo pixel a pixel (el de public/brand/favicon.svg). */
static const char ICONO[] = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Cstyle%3E.suave %7B stroke: %23141414; %7D.px %7B fill: %23141414; display: none; %7D@media (prefers-color-scheme: dark) %7B .suave %7B stroke: %23ededed; %7D .px %7B fill: %23ededed; %7D %7D@media (max-width: 29.99px) %7B .suave %7B display: none; %7D %7D@media (max-width: 17.99px) %7B .p16 %7B display: inline; %7D %7D@media (min-width: 18px) and (max-width: 21.99px) %7B .p20 %7B display: inline; %7D %7D@media (min-width: 22px) and (max-width: 25.99px) %7B .p24 %7B display: inline; %7D %7D@media (min-width: 26px) and (max-width: 29.99px) %7B .p28 %7B display: inline; %7D %7D%3C/style%3E%3Cg fill='none' stroke-width='3.6' stroke-linecap='round' stroke-linejoin='round' class='suave'%3E%3Cpath d='M16 4.5L27.5 16L16 27.5L4.5 16Z'/%3E%3Cpath d='M21.75 10.25L29.4 2.6M10.25 21.75L2.6 29.4'/%3E%3C/g%3E%3Cpath class='px p16' shape-rendering='crispEdges' d='M30 0H32V2H30zM28 2H32V4H28zM14 4H18V6H14zM26 4H30V6H26zM12 6H20V8H12zM24 6H28V8H24zM10 8H14V10H10zM18 8H26V10H18zM8 10H12V12H8zM20 10H24V12H20zM6 12H10V14H6zM22 12H26V14H22zM4 14H8V16H4zM24 14H28V16H24zM4 16H8V18H4zM24 16H28V18H24zM6 18H10V20H6zM22 18H26V20H22zM8 20H12V22H8zM20 20H24V22H20zM6 22H14V24H6zM18 22H22V24H18zM4 24H8V26H4zM12 24H20V26H12zM2 26H6V28H2zM14 26H18V28H14zM0 28H4V30H0zM0 30H2V32H0z'/%3E%3Cpath class='px p20' shape-rendering='crispEdges' d='M30.4 0H32V1.6H30.4zM28.8 1.6H32V3.2H28.8zM27.2 3.2H30.4V4.8H27.2zM14.4 4.8H17.6V6.4H14.4zM25.6 4.8H28.8V6.4H25.6zM12.8 6.4H19.2V8H12.8zM24 6.4H27.2V8H24zM11.2 8H14.4V9.6H11.2zM17.6 8H20.8V9.6H17.6zM22.4 8H25.6V9.6H22.4zM9.6 9.6H12.8V11.2H9.6zM19.2 9.6H24V11.2H19.2zM8 11.2H11.2V12.8H8zM20.8 11.2H24V12.8H20.8zM6.4 12.8H9.6V14.4H6.4zM22.4 12.8H25.6V14.4H22.4zM4.8 14.4H8V16H4.8zM24 14.4H27.2V16H24zM4.8 16H8V17.6H4.8zM24 16H27.2V17.6H24zM6.4 17.6H9.6V19.2H6.4zM22.4 17.6H25.6V19.2H22.4zM8 19.2H11.2V20.8H8zM20.8 19.2H24V20.8H20.8zM8 20.8H12.8V22.4H8zM19.2 20.8H22.4V22.4H19.2zM6.4 22.4H9.6V24H6.4zM11.2 22.4H14.4V24H11.2zM17.6 22.4H20.8V24H17.6zM4.8 24H8V25.6H4.8zM12.8 24H19.2V25.6H12.8zM3.2 25.6H6.4V27.2H3.2zM14.4 25.6H17.6V27.2H14.4zM1.6 27.2H4.8V28.8H1.6zM0 28.8H3.2V30.4H0zM0 30.4H1.6V32H0z'/%3E%3Cpath class='px p24' shape-rendering='crispEdges' d='M30.6667 0H32V1.3333H30.6667zM29.3333 1.3333H32V2.6667H29.3333zM28 2.6667H32V4H28zM14.6667 4H17.3333V5.3333H14.6667zM26.6667 4H30.6667V5.3333H26.6667zM13.3333 5.3333H18.6667V6.6667H13.3333zM25.3333 5.3333H29.3333V6.6667H25.3333zM12 6.6667H20V8H12zM24 6.6667H28V8H24zM10.6667 8H14.6667V9.3333H10.6667zM17.3333 8H21.3333V9.3333H17.3333zM22.6667 8H26.6667V9.3333H22.6667zM9.3333 9.3333H13.3333V10.6667H9.3333zM18.6667 9.3333H25.3333V10.6667H18.6667zM8 10.6667H12V12H8zM20 10.6667H24V12H20zM6.6667 12H10.6667V13.3333H6.6667zM21.3333 12H25.3333V13.3333H21.3333zM5.3333 13.3333H9.3333V14.6667H5.3333zM22.6667 13.3333H26.6667V14.6667H22.6667zM4 14.6667H8V16H4zM24 14.6667H28V16H24zM4 16H8V17.3333H4zM24 16H28V17.3333H24zM5.3333 17.3333H9.3333V18.6667H5.3333zM22.6667 17.3333H26.6667V18.6667H22.6667zM6.6667 18.6667H10.6667V20H6.6667zM21.3333 18.6667H25.3333V20H21.3333zM8 20H12V21.3333H8zM20 20H24V21.3333H20zM6.6667 21.3333H13.3333V22.6667H6.6667zM18.6667 21.3333H22.6667V22.6667H18.6667zM5.3333 22.6667H9.3333V24H5.3333zM10.6667 22.6667H14.6667V24H10.6667zM17.3333 22.6667H21.3333V24H17.3333zM4 24H8V25.3333H4zM12 24H20V25.3333H12zM2.6667 25.3333H6.6667V26.6667H2.6667zM13.3333 25.3333H18.6667V26.6667H13.3333zM1.3333 26.6667H5.3333V28H1.3333zM14.6667 26.6667H17.3333V28H14.6667zM0 28H4V29.3333H0zM0 29.3333H2.6667V30.6667H0zM0 30.6667H1.3333V32H0z'/%3E%3Cpath class='px p28' shape-rendering='crispEdges' d='M30.8571 0H32V1.1429H30.8571zM29.7143 1.1429H32V2.2857H29.7143zM28.5714 2.2857H32V3.4286H28.5714zM27.4286 3.4286H30.8571V4.5714H27.4286zM14.8571 4.5714H17.1429V5.7143H14.8571zM26.2857 4.5714H29.7143V5.7143H26.2857zM13.7143 5.7143H18.2857V6.8571H13.7143zM25.1429 5.7143H28.5714V6.8571H25.1429zM12.5714 6.8571H19.4286V8H12.5714zM24 6.8571H27.4286V8H24zM11.4286 8H14.8571V9.1429H11.4286zM17.1429 8H20.5714V9.1429H17.1429zM22.8571 8H26.2857V9.1429H22.8571zM10.2857 9.1429H13.7143V10.2857H10.2857zM18.2857 9.1429H25.1429V10.2857H18.2857zM9.1429 10.2857H12.5714V11.4286H9.1429zM19.4286 10.2857H24V11.4286H19.4286zM8 11.4286H11.4286V12.5714H8zM20.5714 11.4286H24V12.5714H20.5714zM6.8571 12.5714H10.2857V13.7143H6.8571zM21.7143 12.5714H25.1429V13.7143H21.7143zM5.7143 13.7143H9.1429V14.8571H5.7143zM22.8571 13.7143H26.2857V14.8571H22.8571zM4.5714 14.8571H8V16H4.5714zM24 14.8571H27.4286V16H24zM4.5714 16H8V17.1429H4.5714zM24 16H27.4286V17.1429H24zM5.7143 17.1429H9.1429V18.2857H5.7143zM22.8571 17.1429H26.2857V18.2857H22.8571zM6.8571 18.2857H10.2857V19.4286H6.8571zM21.7143 18.2857H25.1429V19.4286H21.7143zM8 19.4286H11.4286V20.5714H8zM20.5714 19.4286H24V20.5714H20.5714zM8 20.5714H12.5714V21.7143H8zM19.4286 20.5714H22.8571V21.7143H19.4286zM6.8571 21.7143H13.7143V22.8571H6.8571zM18.2857 21.7143H21.7143V22.8571H18.2857zM5.7143 22.8571H9.1429V24H5.7143zM11.4286 22.8571H14.8571V24H11.4286zM17.1429 22.8571H20.5714V24H17.1429zM4.5714 24H8V25.1429H4.5714zM12.5714 24H19.4286V25.1429H12.5714zM3.4286 25.1429H6.8571V26.2857H3.4286zM13.7143 25.1429H18.2857V26.2857H13.7143zM2.2857 26.2857H5.7143V27.4286H2.2857zM14.8571 26.2857H17.1429V27.4286H14.8571zM1.1429 27.4286H4.5714V28.5714H1.1429zM0 28.5714H3.4286V29.7143H0zM0 29.7143H2.2857V30.8571H0zM0 30.8571H1.1429V32H0z'/%3E%3C/svg%3E";

/**
 * Las dos trazas de la bienvenida, corriendo: la digital en azul y la
 * analógica en violeta. Cada una es más larga que la vista por un período
 * entero, así que correrla exactamente un período la deja donde empezó y la
 * vuelta no se nota.
 */
static const char ONDAS[] = R"SVG(<svg class="ondas" viewBox="0 0 400 64" preserveAspectRatio="none" aria-hidden="true">
<g class="corre" stroke="#3b9eff"><path class="halo" d="M-120 30H-60V12H0V30H60V12H120V30H180V12H240V30H300V12H360V30H420V12H480"/><path class="traza" d="M-120 30H-60V12H0V30H60V12H120V30H180V12H240V30H300V12H360V30H420V12H480"/></g>
<g class="lenta" stroke="#c084fc"><path class="halo" d="M-160 46C-133 32-107 32-80 46S-27 60 0 46S53 32 80 46S133 60 160 46S213 32 240 46S293 60 320 46S373 32 400 46S453 60 480 46S533 32 560 46"/><path class="traza" d="M-160 46C-133 32-107 32-80 46S-27 60 0 46S53 32 80 46S133 60 160 46S213 32 240 46S293 60 320 46S373 32 400 46S453 60 480 46S533 32 560 46"/></g>
</svg>)SVG";

/** Lista de redes cercanas, validación del formulario y el ojo de la clave. */
static const char SCRIPT_WIFI[] = R"JS(<script>
(function(){
  var lista = document.getElementById('redes');
  var nombre = document.getElementById('s');
  var clave = document.getElementById('p');
  var aviso = document.getElementById('aviso');
  var elegida = nombre.value;
  var CANDADO = '<svg class="candado" viewBox="0 0 16 16" aria-hidden="true"><rect x="3.5" y="7" width="9" height="6.5" rx="1.5"/><path d="M5.5 7V5.2a2.5 2.5 0 0 1 5 0V7"/></svg>';

  function barras(n){
    var s = '<svg viewBox="0 0 16 16" aria-hidden="true">';
    for (var i = 0; i < 4; i++) {
      var h = 4 + i * 3.5;
      s += '<rect x="' + (1 + i * 3.8) + '" y="' + (15 - h) + '" width="2.6" height="' + h + '" rx="1"' + (i < n ? '' : ' class="apagada"') + '/>';
    }
    return s + '</svg>';
  }
  function nivel(r){ return r > -55 ? 4 : r > -67 ? 3 : r > -78 ? 2 : 1; }
  function mensaje(t, buscando){
    lista.innerHTML = '<p class="vacio">' + (buscando ? '<span class="punto espera pulso"></span>' : '') + '<span></span></p>';
    lista.querySelector('span:last-child').textContent = t;
  }
  function decir(t){ aviso.textContent = t; aviso.hidden = !t; }

  function pintar(redes){
    if (!redes.length) { mensaje('No se encontró ninguna red. Puedes escribir el nombre a mano.'); return; }
    lista.innerHTML = '';
    redes.forEach(function(r){
      var b = document.createElement('button');
      b.type = 'button';
      b.className = 'red' + (r.s === elegida ? ' sel' : '');
      b.innerHTML = barras(nivel(r.r)) + '<span></span>' + (r.c ? CANDADO : '');
      b.querySelector('span').textContent = r.s;
      b.title = r.c ? 'Red con clave' : 'Red abierta';
      b.onclick = function(){
        elegida = r.s;
        nombre.value = r.s;
        Array.prototype.forEach.call(lista.children, function(x){ x.classList.remove('sel'); });
        b.classList.add('sel');
        decir('');
        if (r.c) clave.focus(); else clave.value = '';
      };
      lista.appendChild(b);
    });
  }

  function buscar(){
    var intentos = 0;
    mensaje('Buscando redes…', true);
    (function consultar(){
      fetch('/scan', { cache: 'no-store' })
        .then(function(r){ return r.json(); })
        .then(function(d){
          if (d.buscando) {
            if (++intentos > 25) { mensaje('No se pudo buscar redes. Escribe el nombre a mano.'); return; }
            setTimeout(consultar, 800);
            return;
          }
          pintar(d.redes || []);
        })
        .catch(function(){ mensaje('No se pudo buscar redes. Escribe el nombre a mano.'); });
    })();
  }

  document.getElementById('buscar').onclick = buscar;
  nombre.oninput = function(){
    elegida = nombre.value;
    Array.prototype.forEach.call(lista.children, function(x){
      var s = x.querySelector && x.querySelector('span');
      if (x.classList.contains('red')) x.classList.toggle('sel', !!s && s.textContent === elegida);
    });
  };
  document.getElementById('ver').onclick = function(){
    var oculta = clave.type === 'password';
    clave.type = oculta ? 'text' : 'password';
    this.textContent = oculta ? 'Ocultar' : 'Mostrar';
    clave.focus();
  };
  document.getElementById('form').onsubmit = function(e){
    var n = clave.value.length;
    if (!nombre.value.trim()) { e.preventDefault(); decir('Elige una red de la lista o escribe su nombre.'); nombre.focus(); return; }
    if (n > 0 && n < 8) { e.preventDefault(); decir('Una clave WiFi tiene al menos 8 caracteres.'); clave.focus(); return; }
    var g = document.getElementById('guardar');
    g.disabled = true;
    g.textContent = 'Guardando…';
  };
  buscar();
})();
</script>)JS";

/**
 * Cuerpo del puente.
 *
 * Además de pasar mensajes, es el buscador de equipos: pregunta cada tanto a
 * su propio equipo por /peers —que conoce a todos los de la red— y le pasa la
 * lista a la app. El WebSocket no va necesariamente a este equipo: la app
 * elige uno de la lista y el puente lo abre contra esa dirección. Desde acá
 * eso está permitido, porque es una página http hablando con otra dirección
 * de la misma red. Soltar un equipo no cierra la ventana: se sigue buscando y
 * se puede elegir otro.
 *
 * Cada conexión lleva el número `n` que le puso la app, y todo lo que el
 * puente cuenta de ella vuelve con ese número: así un aviso tardío de un
 * equipo que ya se soltó no se confunde con el que está en curso.
 */
static const char CUERPO_PUENTE[] = R"HTML(<p class="rotulo">Enlace WiFi</p>
<h1 id="t">Conectando con la app</h1>
<div class="caja">
<div class="fila"><span>Estado</span><span class="estado"><span id="punto" class="punto espera pulso"></span><span id="e">Abriendo…</span></span></div>
<div class="fila"><span>Equipos en la red</span><span id="c">—</span></div>
</div>
<p class="nota" id="n">Deja esta ventana abierta mientras uses los equipos. Si la cierras, el enlace se corta y la señal se detiene.</p>
<script>
(function(){
  // La app pone su propio origen en el hash al abrir esta ventana: es contra
  // eso que se valida cada mensaje, en los dos sentidos.
  var destino = decodeURIComponent((location.hash || '').slice(1));
  var app = window.opener;
  var ws = null, wsNombre = '';
  var mirando = false, fallos = 0, cuantos = -1;

  function decir(titulo, estado, clase){
    document.getElementById('t').textContent = titulo;
    document.getElementById('e').textContent = estado;
    document.getElementById('punto').className = 'punto ' + clase;
  }
  function post(m){
    if (!app || app.closed) return;
    m.oryx = 'bridge';
    try { app.postMessage(m, destino || '*'); } catch (err) {}
  }
  function pintar(){
    document.getElementById('c').textContent = cuantos < 0 ? '—' : String(cuantos);
    if (ws && ws.readyState === 1) decir('Enlace activo', 'Conectado a ' + wsNombre, 'ok');
    else if (ws)                   decir('Conectando', 'Abriendo ' + wsNombre + '…', 'espera pulso');
    else                           decir('Buscando equipos', 'Elige uno en la app', 'espera pulso');
  }

  // Pregunta a su equipo quién anda en la red, sin pausa mientras la ventana
  // viva. Con un equipo conectado alcanza con más calma: la lista solo sirve
  // para cuando se vuelva a elegir.
  function consultar(){
    fetch('/peers', { cache: 'no-store' })
      .then(function(r){ return r.json(); })
      .then(function(d){
        fallos = 0;
        var lista = d.list || [];
        cuantos = lista.length;
        post({ t: 'peers', self: d.self, list: lista });
        pintar();
      })
      .catch(function(){
        // Su propio equipo dejó de contestar: la app buscará por otro lado.
        if (++fallos >= 3) post({ t: 'lost' });
      })
      .then(function(){ setTimeout(consultar, ws ? 5000 : 2000); });
  }

  function soltar(){
    if (!ws) return;
    var w = ws;
    ws = null;
    w.onopen = w.onmessage = w.onerror = w.onclose = null;
    try { w.close(); } catch (err) {}
  }

  function abrir(host, n, nombre){
    soltar();
    wsNombre = nombre || host;
    var w;
    try { w = new WebSocket('ws://' + host + ':81/'); }
    catch (err) { post({ t: 'error', n: n }); return; }
    ws = w;
    w.onopen    = function(){ if (ws === w) { pintar(); post({ t: 'open', n: n }); } };
    w.onmessage = function(ev){ if (ws === w) post({ t: 'rx', n: n, d: ev.data }); };
    w.onerror   = function(){ if (ws === w) post({ t: 'error', n: n }); };
    w.onclose   = function(){ if (ws === w) { ws = null; pintar(); post({ t: 'closed', n: n }); } };
    pintar();
  }

  window.addEventListener('message', function(ev){
    if (destino && ev.origin !== destino) return;
    if (app && ev.source !== app) return;
    var d = ev.data;
    if (!d || d.oryx !== 'app') return;
    if (d.t === 'hello') {
      post({ t: 'up', v: 2 });
      if (!mirando) { mirando = true; pintar(); consultar(); }
    }
    else if (d.t === 'connect') { abrir(String(d.host || location.hostname), d.n, d.name); }
    else if (d.t === 'drop')    { soltar(); pintar(); }
    else if (d.t === 'tx')      { if (ws && ws.readyState === 1) ws.send(d.d); }
    else if (d.t === 'bye')     { soltar(); window.close(); }
  });

  window.addEventListener('beforeunload', function(){ soltar(); post({ t: 'gone' }); });

  if (!app) {
    decir('Abre el enlace desde la app', 'Sin app', 'error');
    document.getElementById('n').textContent = 'Esta ventana la abre Oryx Signal Studio desde Dispositivo → WiFi. Entrar a mano no conecta nada.';
  } else {
    post({ t: 'up', v: 2 });
  }
})();
</script>)HTML";

// ═══════════════════════════════════════════════════════════════════════════
// RED Y PUERTO SERIE — núcleo 0
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Toma la próxima conexión que esté esperando.
 *
 * El core 3.x renombró `available()` a `accept()` y deja el nombre viejo
 * marcado como obsoleto; el 2.x solo tiene el viejo.
 */
static WiFiClient aceptar(WiFiServer& servidor) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  return servidor.accept();
#else
  return servidor.available();
#endif
}

static WiFiServer  servidorHttp(80);
static WiFiServer  servidorWs(81);
static WiFiClient  clienteWs;
static bool        wsAbierto = false;

/** Contesta cualquier nombre con la dirección propia: es lo que vuelve cautivo al portal. */
static DNSServer   dns;

static Preferences ajustes;
static String      staSsid;
static String      staClave;

/** Cómo va la red de casa. */
enum FaseRed : uint8_t {
  RED_SIN_CONFIGURAR,   // no hay ninguna guardada
  RED_ENTRANDO,         // probando, hasta PLAZO_RED_MS
  RED_ADENTRO,          // conectado
  RED_ESPERANDO,        // no se pudo; se vuelve a probar cada REINTENTO_RED_MS
};
static FaseRed  faseRed  = RED_SIN_CONFIGURAR;
static uint32_t faseDesde = 0;

static char   rx[RX_BUF];
static size_t rxLargo = 0;

/** Enlace por cable: se enciende con la primera orden y se apaga por silencio. */
static bool     enlaceSerie  = false;
static uint32_t ultimaSerie  = 0;
static char     rxSerie[512];
static size_t   rxSerieLargo = 0;

/** Carga a medio recibir: entre BEGIN y END. */
static bool     cargando    = false;
static uint16_t filasVistas = 0;

/**
 * Diagnóstico para quien mire el monitor serie.
 *
 * Va con almohadilla adelante porque por el cable comparte camino con las
 * respuestas del protocolo: así la app las distingue de un vistazo y las
 * ignora en vez de intentar interpretarlas.
 */
static void nota(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(F("# "));
  Serial.println(buf);
}

/** Respuesta para quien esté escuchando, sea por WiFi o por cable. */
static void decir(const char* linea) {
  if (wsAbierto)   wsSendText(clienteWs, linea);
  if (enlaceSerie) { Serial.print(linea); Serial.print('\n'); }
}

static void decirf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  decir(buf);
}

static void mandarEstado() {
  decirf("STATE %u %.3f %u %u %.3f %u %u %u %u",
         corriendo ? 1u : 0u,
         posActual,
         ciclos,
         mv,
         (double)usPaso,
         rpm,
         modo == MODO_DIGITAL ? (uint32_t)nFilas : nMuestras,
         (uint32_t)modo,
         ciclosSenal);
}

static void mandarSaludo() {
  decirf("READY %u %s %u %u %u %u %u",
         (uint32_t)PROTO, nombreEquipo,
         (uint32_t)MAX_CANALES_D, (uint32_t)MAX_CANALES_A,
         (uint32_t)MAX_FILAS, (uint32_t)MAX_MUESTRAS,
         V_MAX_MV);
}

// ── Comandos ───────────────────────────────────────────────────────────────

/** Arranca una carga nueva: BEGIN <D|A> <canales> <pasos> <bits> <spp> */
static void cmdBegin(char* args) {
  char* resto = nullptr;
  const char* tipo  = strtok_r(args, " ", &resto);
  const char* sCan  = strtok_r(nullptr, " ", &resto);
  const char* sPas  = strtok_r(nullptr, " ", &resto);
  const char* sBits = strtok_r(nullptr, " ", &resto);
  const char* sSpp  = strtok_r(nullptr, " ", &resto);
  if (!tipo || !sCan || !sPas) { decir("ERR BEGIN incompleto"); return; }

  detenerYEsperar();

  const uint8_t nuevoModo = (tipo[0] == 'A') ? MODO_ANALOGICO : MODO_DIGITAL;
  const uint8_t tope = nuevoModo == MODO_ANALOGICO ? MAX_CANALES_A : MAX_CANALES_D;
  uint8_t canales = (uint8_t)atoi(sCan);
  if (canales == 0) { decir("ERR sin canales"); return; }
  if (canales > tope) canales = tope;

  modo         = nuevoModo;
  nCanales     = canales;
  pasosTotales = (float)atof(sPas);
  nivelMax     = sBits ? ((1UL << (uint8_t)atoi(sBits)) - 1) : 4095;
  spp          = sSpp ? (uint8_t)atoi(sSpp) : 16;
  if (spp == 0) spp = 1;
  nFilas       = 0;
  nMuestras    = 0;
  filasVistas  = 0;
  ciclos       = 0;
  posActual    = 0;
  cargando     = true;
  recalcularFactor();
  salidasIdle();
}

/** Una fila digital: D <paso> <mascara> */
static void cmdFila(char* args) {
  if (!cargando || modo != MODO_DIGITAL) return;
  char* resto = nullptr;
  const char* sPos  = strtok_r(args, " ", &resto);
  const char* sMask = strtok_r(nullptr, " ", &resto);
  if (!sPos || !sMask) return;
  filasVistas++;
  if (nFilas >= MAX_FILAS) return;
  filas[nFilas].pos  = (float)atof(sPos);
  filas[nFilas].mask = (uint16_t)strtoul(sMask, nullptr, 10);
  nFilas++;
}

/** Un tramo de muestras: A <canal> <indice> <v,v,v,…> */
static void cmdMuestras(char* args) {
  if (!cargando || modo != MODO_ANALOGICO) return;
  char* resto = nullptr;
  const char* sCan = strtok_r(args, " ", &resto);
  const char* sIdx = strtok_r(nullptr, " ", &resto);
  char* lista      = strtok_r(nullptr, " ", &resto);
  if (!sCan || !sIdx || !lista) return;

  const uint8_t canal = (uint8_t)atoi(sCan);
  uint32_t i = (uint32_t)strtoul(sIdx, nullptr, 10);
  if (canal >= MAX_CANALES_A) return;

  char* guarda = nullptr;
  for (char* tok = strtok_r(lista, ",", &guarda); tok; tok = strtok_r(nullptr, ",", &guarda)) {
    if (i < MAX_MUESTRAS) muestras[canal][i] = (uint16_t)strtoul(tok, nullptr, 10);
    i++;
    if (i > nMuestras) nMuestras = i > MAX_MUESTRAS ? MAX_MUESTRAS : i;
  }
}

/** Cierra la carga: END <filas esperadas> */
static void cmdEnd(char* args) {
  cargando = false;
  const uint32_t esperadas = args ? strtoul(args, nullptr, 10) : 0;
  if (modo == MODO_DIGITAL && esperadas > 0 && filasVistas != esperadas) {
    decirf("ERR llegaron %u de %u filas", (uint32_t)filasVistas, esperadas);
    return;
  }
  if (modo == MODO_DIGITAL && filasVistas > MAX_FILAS) {
    decirf("ERR la señal tiene %u cambios y entran %u",
           (uint32_t)filasVistas, (uint32_t)MAX_FILAS);
    return;
  }
  if (pasosTotales <= 0) { decir("ERR la señal no tiene largo"); return; }

  if (modo == MODO_DIGITAL) prepararFilas();
  recalcularFactor();
  resolverRitmo();
  pedidoReinicio = true;
  decirf("LOADED %u", modo == MODO_DIGITAL ? (uint32_t)nFilas : nMuestras);
  mandarEstado();
}

/** Parámetros: SET V <mv> · SET T <us> · SET R <rpm> · SET C <ciclos> */
static void cmdSet(char* args) {
  char* resto = nullptr;
  const char* que = strtok_r(args, " ", &resto);
  const char* val = strtok_r(nullptr, " ", &resto);
  if (!que || !val) { decir("ERR SET incompleto"); return; }
  const uint32_t v = strtoul(val, nullptr, 10);

  switch (que[0]) {
    case 'V':
      mv = v > V_MAX_MV ? V_MAX_MV : v;
      recalcularFactor();
      if (modo != MODO_ANALOGICO) escribirReferencia();
      break;
    case 'T':
      usPedido = v < 5 ? 5 : v;
      resolverRitmo();
      break;
    case 'R':
      rpm = v > RPM_MAX ? RPM_MAX : v;
      resolverRitmo();
      break;
    case 'C':
      ciclosSenal = v < CICLOS_MIN ? CICLOS_MIN : (v > CICLOS_MAX ? CICLOS_MAX : v);
      break;
    default:
      decir("ERR parámetro desconocido");
      return;
  }
  mandarEstado();
}

static void procesarLinea(char* linea) {
  while (*linea == ' ') linea++;
  if (*linea == '\0' || *linea == '\r') return;

  char* args = strchr(linea, ' ');
  if (args) { *args = '\0'; args++; }

  if      (!strcmp(linea, "D"))     cmdFila(args);
  else if (!strcmp(linea, "A"))     cmdMuestras(args);
  else if (!strcmp(linea, "BEGIN")) cmdBegin(args);
  else if (!strcmp(linea, "END"))   cmdEnd(args);
  else if (!strcmp(linea, "SET"))   cmdSet(args);
  else if (!strcmp(linea, "HELLO")) { mandarSaludo(); mandarEstado(); }
  else if (!strcmp(linea, "PING"))  decirf("PONG %s", args ? args : "0");
  else if (!strcmp(linea, "RUN")) {
    if (cargando)          { decir("ERR la señal todavía se está cargando"); return; }
    if (modo == MODO_NADA) { decir("ERR todavía no hay señal cargada"); return; }
    corriendo = true;
    mandarEstado();
  }
  else if (!strcmp(linea, "HALT")) {
    detenerYEsperar();
    salidasIdle();
    mandarEstado();
  }
  else if (!strcmp(linea, "RESET")) {
    pedidoReinicio = true;
    ciclos = 0;
    posActual = 0;
    if (!corriendo) salidasIdle();
    mandarEstado();
  }
  else decirf("ERR orden desconocida: %s", linea);
}

/** Un mensaje del WebSocket puede traer varias órdenes, una por línea. */
static void procesarMensaje(char* texto) {
  char* guarda = nullptr;
  for (char* l = strtok_r(texto, "\n", &guarda); l; l = strtok_r(nullptr, "\n", &guarda)) {
    procesarLinea(l);
  }
}

// ── Puerto serie ───────────────────────────────────────────────────────────

/**
 * Órdenes que llegan por el cable, una por línea.
 *
 * Se vacía todo lo que haya en cada pasada. El buffer de entrada es grande a
 * propósito (ver RX_SERIE): la señal llega como un chorro largo y sin control
 * de flujo, así que lo único que evita perder bytes es leer seguido y tener
 * lugar donde esperarlos.
 */
static void atenderSerie() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (rxSerieLargo > 0) {
        rxSerie[rxSerieLargo] = '\0';
        rxSerieLargo = 0;
        enlaceSerie = true;
        ultimaSerie = millis();
        procesarLinea(rxSerie);
      }
      continue;
    }
    if (rxSerieLargo < sizeof(rxSerie) - 1) rxSerie[rxSerieLargo++] = ch;
    else rxSerieLargo = 0;                 // línea imposible: se descarta entera
  }

  // La app manda un PING cada dos segundos. Si dejó de llegar, cerraron la
  // pestaña o desenchufaron: la señal se para, igual que al caerse el WiFi.
  if (enlaceSerie && millis() - ultimaSerie > SILENCIO_SERIE_MS) {
    enlaceSerie = false;
    rxSerieLargo = 0;
    cargando = false;
    detenerYEsperar();
    salidasIdle();
    nota("Enlace por cable cerrado por silencio");
  }
}

// ── Red de casa ────────────────────────────────────────────────────────────

static void entrarARed() {
  WiFi.begin(staSsid.c_str(), staClave.c_str());
  faseRed = RED_ENTRANDO;
  faseDesde = millis();
}

/**
 * Sigue a la red de casa sin molestar a la propia.
 *
 * Mientras el ESP32 busca un router salta de canal, y la red propia salta con
 * él: quien esté conectado a ella se cae. Por eso un intento fallido se corta
 * a los PLAZO_RED_MS, y los reintentos solo se hacen cuando no hay nadie en la
 * red propia —que es justamente cuando alguien puede estar usando el portal.
 */
static void vigilarRed() {
  const bool adentro = WiFi.status() == WL_CONNECTED;
  switch (faseRed) {
    case RED_SIN_CONFIGURAR:
      break;
    case RED_ENTRANDO:
      if (adentro) {
        faseRed = RED_ADENTRO;
        nota("En %s como %s  ·  http://%s.local/",
             staSsid.c_str(), WiFi.localIP().toString().c_str(), MDNS_NOMBRE);
      } else if (millis() - faseDesde > PLAZO_RED_MS) {
        WiFi.disconnect(false);
        faseRed = RED_ESPERANDO;
        faseDesde = millis();
        nota("No se pudo entrar a %s; queda la red propia", staSsid.c_str());
      }
      break;
    case RED_ADENTRO:
      // El core reintenta solo; si en el plazo no vuelve, se pasa a esperar.
      if (!adentro) {
        faseRed = RED_ENTRANDO;
        faseDesde = millis();
        nota("Se perdió la red %s", staSsid.c_str());
      }
      break;
    case RED_ESPERANDO:
      if (millis() - faseDesde > REINTENTO_RED_MS && WiFi.softAPgetStationNum() == 0) entrarARed();
      break;
  }
}

// ── Vecinos ────────────────────────────────────────────────────────────────
//
// Un navegador no puede recorrer la red buscando equipos: no habla mDNS y una
// página https ni siquiera puede asomarse a una dirección local. El ESP32 sí.
// Cada equipo se anuncia como _oryx._tcp y, mientras alguien mira la lista,
// pregunta quién más se anuncia igual. La app entra por cualquiera de ellos
// (oryx-signal.local, la red propia, o el último que contestó) y recibe la
// lista de todos en /peers.

struct Vecino {
  char     id[8];
  char     nombre[24];
  char     ip[16];
  bool     ocupado;
  uint32_t visto;
};

static Vecino            vecinos[MAX_VECINOS];
static uint8_t           nVecinos = 0;
static SemaphoreHandle_t candadoVecinos = nullptr;

/** Última vez que alguien pidió la lista. Sin nadie mirando no se busca. */
static volatile uint32_t ultimaMirada = 0;

/** Lo que se anuncia de este equipo mientras está en uso, para que los demás lo muestren así. */
static bool ocupadoAnunciado = false;

static void anunciarOcupado(bool ocupado) {
  if (ocupado == ocupadoAnunciado) return;
  ocupadoAnunciado = ocupado;
  mdns_service_txt_item_set(MDNS_SERVICIO, MDNS_PROTO, "busy", ocupado ? "1" : "0");
}

/** Copia solo lo que puede ir sin escapar en JSON y en HTML: letras, números y guiones. */
static void copiarLimpio(char* dst, size_t cap, const char* src) {
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j + 1 < cap; i++) {
    const char c = src[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.') dst[j++] = c;
  }
  dst[j] = '\0';
}

static const char* txt(const mdns_result_t* r, const char* clave) {
  for (size_t i = 0; i < r->txt_count; i++) {
    if (r->txt[i].key && !strcmp(r->txt[i].key, clave)) return r->txt[i].value ? r->txt[i].value : "";
  }
  return nullptr;
}

/** Una vuelta de búsqueda: suma o refresca a quien contestó y olvida a quien se calló. */
static void buscarVecinos() {
  mdns_result_t* resultados = nullptr;
  if (mdns_query_ptr(MDNS_SERVICIO, MDNS_PROTO, BUSQUEDA_PLAZO_MS, MAX_VECINOS * 2, &resultados) != ESP_OK) return;

  const uint32_t ahora = millis();
  xSemaphoreTake(candadoVecinos, portMAX_DELAY);
  for (mdns_result_t* r = resultados; r; r = r->next) {
    const char* id = txt(r, "id");
    if (!id || !strcmp(id, idEquipo)) continue;          // uno mismo no es vecino

    // Solo IPv4: es la dirección a la que después apunta el WebSocket.
    char ip[16] = { 0 };
    for (mdns_ip_addr_t* a = r->addr; a; a = a->next) {
      if (a->addr.type == ESP_IPADDR_TYPE_V4) {
        strncpy(ip, IPAddress(a->addr.u_addr.ip4.addr).toString().c_str(), sizeof(ip) - 1);
        break;
      }
    }
    if (!ip[0]) continue;

    uint8_t k = 0;
    while (k < nVecinos && strcmp(vecinos[k].id, id)) k++;
    if (k == nVecinos) {
      if (nVecinos >= MAX_VECINOS) continue;
      nVecinos++;
    }
    Vecino& v = vecinos[k];
    copiarLimpio(v.id, sizeof(v.id), id);
    copiarLimpio(v.nombre, sizeof(v.nombre), r->instance_name ? r->instance_name : id);
    strncpy(v.ip, ip, sizeof(v.ip) - 1);
    v.ip[sizeof(v.ip) - 1] = '\0';
    const char* ocupado = txt(r, "busy");
    v.ocupado = ocupado && ocupado[0] == '1';
    v.visto = ahora;
  }

  // Los que no contestaron hace rato se van, corriendo el último a su lugar.
  for (uint8_t k = 0; k < nVecinos;) {
    if (ahora - vecinos[k].visto > VECINO_VIGENCIA_MS) vecinos[k] = vecinos[--nVecinos];
    else k++;
  }
  xSemaphoreGive(candadoVecinos);
  mdns_query_results_free(resultados);
}

/**
 * Busca vecinos en su propia tarea.
 *
 * La consulta espera respuestas durante BUSQUEDA_PLAZO_MS: hecha en la tarea
 * de red dejaría al WebSocket y al puerto serie sin atender todo ese rato.
 */
static void tareaVecinos(void*) {
  for (;;) {
    if (faseRed == RED_ADENTRO && millis() - ultimaMirada < MIRADA_MS) buscarVecinos();
    vTaskDelay(pdMS_TO_TICKS(BUSQUEDA_CADA_MS));
  }
}

// ── Páginas ────────────────────────────────────────────────────────────────

/** Texto listo para meter en HTML: un nombre de red puede traer cualquier cosa. */
static String html(const String& t) {
  String o;
  o.reserve(t.length() + 8);
  for (size_t i = 0; i < t.length(); i++) {
    const char c = t[i];
    switch (c) {
      case '&':  o += F("&amp;");  break;
      case '<':  o += F("&lt;");   break;
      case '>':  o += F("&gt;");   break;
      case '"':  o += F("&quot;"); break;
      case '\'': o += F("&#39;");  break;
      default:   o += c;
    }
  }
  return o;
}

/** Texto listo para meter entre comillas en JSON. */
static String json(const String& t) {
  String o;
  o.reserve(t.length() + 4);
  for (size_t i = 0; i < t.length(); i++) {
    const char c = t[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { char u[8]; snprintf(u, sizeof(u), "\\u%04x", (uint8_t)c); o += u; }
    else o += c;
  }
  return o;
}

/** Cabecera de marca y comienzo de la tarjeta. `ondas` suma las trazas. */
static String abrirPagina(const char* titulo, bool ondas) {
  String s;
  s.reserve(15000);  // el ícono de la pestaña solo ya ocupa unos 7 KB
  s += F("<!doctype html><html lang=\"es\"><head><meta charset=\"utf-8\">"
         "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
         "<meta name=\"color-scheme\" content=\"light dark\"><title>");
  s += titulo;
  s += F(" · Oryx Signal</title><link rel=\"icon\" href=\"");
  s += ICONO;
  s += F("\">");
  s += ESTILO;
  s += F("</head><body><main class=\"tarjeta\"><header class=\"cabecera");
  if (ondas) s += F(" con-ondas");
  s += F("\"><div class=\"marca\">");
  s += LOGO;
  s += F("<span>Oryx Signal Studio</span></div>");
  if (ondas) s += ONDAS;
  s += F("</header><div class=\"filo\"></div><section class=\"cuerpo\">");
  return s;
}

static void cerrarPagina(String& s) {
  s += F("</section></main></body></html>");
}

/** Renglón de la caja de datos. `valor` ya tiene que venir escapado. */
static void fila(String& s, const char* rotulo, const String& valor) {
  s += F("<div class=\"fila\"><span>");
  s += rotulo;
  s += F("</span><span>");
  s += valor;
  s += F("</span></div>");
}

/** Cómo está la red de casa, con su luz. */
static String estadoRed() {
  String s = F("<span class=\"estado\"><span class=\"punto ");
  switch (faseRed) {
    case RED_SIN_CONFIGURAR:
      s += F("\"></span>Sin configurar");
      break;
    case RED_ENTRANDO:
      s += F("espera pulso\"></span>Conectando a ");
      s += html(staSsid);
      s += F("…");
      break;
    case RED_ADENTRO:
      s += F("ok\"></span>");
      s += html(staSsid);
      break;
    case RED_ESPERANDO:
      s += F("error\"></span>No se pudo entrar a ");
      s += html(staSsid);
      break;
  }
  s += F("</span>");
  return s;
}

/** Qué se ve al entrar a la dirección del equipo. */
static String paginaInicio() {
  String s = abrirPagina("Generador ESP32", true);
  s += F("<p class=\"rotulo\">Generador ESP32</p>"
         "<h1>El equipo está encendido</h1>"
         "<p>Reproduce la señal que armas en Oryx Signal Studio, por USB o por WiFi.</p>"
         "<div class=\"caja\">");
  fila(s, "Equipo", String("<code>") + nombreEquipo + "</code>");
  fila(s, "Tu red WiFi", estadoRed());
  if (faseRed == RED_ADENTRO) {
    fila(s, "Dirección en tu red", String("<code>") + WiFi.localIP().toString() + "</code>");
  }
  fila(s, "Red propia", String("<code>") + nombreEquipo + "</code>");
  fila(s, "Dirección propia", String("<code>") + WiFi.softAPIP().toString() + "</code>");
  s += F("</div>");

  if (faseRed == RED_ADENTRO) {
    s += F("<a class=\"boton sec\" style=\"margin-top:24px\" href=\"/wifi\">Cambiar de red WiFi</a>");
  } else {
    s += F("<a class=\"boton\" href=\"/wifi\">Configurar WiFi</a>");
  }
  s += F("<p class=\"nota\">Para usarlo, vuelve a Oryx Signal Studio y abre <b>Dispositivo</b>. "
         "No hace falta abrir nada más desde aquí.</p>");
  cerrarPagina(s);
  return s;
}

/** Portal de configuración: se carga una vez y queda guardado. */
static String paginaWifi(const char* error) {
  String s = abrirPagina("WiFi", false);
  s += F("<p class=\"rotulo\">Configurar WiFi</p>"
         "<h1>Conecta el equipo a tu red</h1>"
         "<p>Así tu computadora no tiene que cambiar de red para hablarle, y no pierde internet. "
         "Queda guardado en el equipo.</p>");

  s += F("<p id=\"aviso\" class=\"aviso\" role=\"alert\"");
  if (error && error[0]) { s += '>'; s += error; }
  else                   { s += F(" hidden>"); }
  s += F("</p>");

  s += F("<div class=\"encabezado\"><span class=\"etiqueta\">Redes cercanas</span>"
         "<button type=\"button\" id=\"buscar\" class=\"chico\">Buscar de nuevo</button></div>"
         "<div id=\"redes\" class=\"redes\" aria-live=\"polite\"></div>"
         "<form id=\"form\" method=\"POST\" action=\"/wifi\" autocomplete=\"off\">"
         "<label class=\"etiqueta\" for=\"s\">Nombre de la red</label>"
         "<div class=\"campo\"><input id=\"s\" name=\"s\" maxlength=\"32\" "
         "autocapitalize=\"none\" spellcheck=\"false\" placeholder=\"Elige una de la lista o escríbela\" value=\"");
  s += html(staSsid);
  s += F("\" style=\"padding-right:14px\"></div>"
         "<label class=\"etiqueta\" for=\"p\">Clave</label>"
         "<div class=\"campo\"><input id=\"p\" name=\"p\" type=\"password\" maxlength=\"63\" "
         "autocapitalize=\"none\" spellcheck=\"false\" placeholder=\"Vacía si la red es abierta\">"
         "<button type=\"button\" id=\"ver\" class=\"ver\">Mostrar</button></div>"
         "<button id=\"guardar\" class=\"boton\" type=\"submit\">Guardar y conectar</button>"
         "</form>");

  if (faseRed != RED_SIN_CONFIGURAR) {
    s += F("<form method=\"POST\" action=\"/olvidar\">"
           "<button class=\"boton sec\" type=\"submit\">Olvidar ");
    s += html(staSsid);
    s += F("</button></form>");
  }

  s += F("<p class=\"nota\">La red propia <code>");
  s += nombreEquipo;
  s += F("</code> existe siempre, así que nunca te quedas sin forma de entrar al equipo. "
         "<a href=\"/\">Volver al inicio</a></p>");
  s += SCRIPT_WIFI;
  cerrarPagina(s);
  return s;
}

static String paginaGuardado(const String& ssid) {
  String s = abrirPagina("Guardado", false);
  s += F("<p class=\"rotulo\">Guardado</p><h1>Conectando a ");
  s += html(ssid);
  s += F("</h1><p>El equipo se reinicia para entrar a tu red. Tarda unos segundos.</p>"
         "<ol class=\"pasos\">"
         "<li>Vuelve a conectar esta computadora a <b>");
  s += html(ssid);
  s += F("</b>.</li>"
         "<li>En Oryx Signal Studio abre <b>Dispositivo → WiFi</b> y elige <code>");
  s += nombreEquipo;
  s += F("</code> en la lista de equipos.</li></ol>"
         "<p class=\"nota\">Si no aparece, vuelve a la red <code>");
  s += nombreEquipo;
  s += F("</code> y entra a <code>");
  s += WiFi.softAPIP().toString();
  s += F("</code>: ahí verás si pudo conectarse y con qué dirección. "
         "El monitor serie también la muestra.</p>");
  cerrarPagina(s);
  return s;
}

static String paginaOlvidada() {
  String s = abrirPagina("Red olvidada", false);
  s += F("<p class=\"rotulo\">Listo</p><h1>Red olvidada</h1>"
         "<p>El equipo se reinicia y queda solo con su red propia, <code>");
  s += nombreEquipo;
  s += F("</code>. Puedes configurar otra cuando quieras.</p>");
  cerrarPagina(s);
  return s;
}

static String paginaPuente() {
  String s = abrirPagina("Enlace", false);
  s += CUERPO_PUENTE;
  cerrarPagina(s);
  return s;
}

/**
 * Redes cercanas, en JSON.
 *
 * La búsqueda corre en segundo plano: la primera consulta la arranca y
 * contesta que está buscando, y la página vuelve a preguntar hasta que haya
 * resultados. Así el servidor nunca se queda trabado los segundos que tarda
 * en recorrer los canales.
 */
static String respuestaBusqueda() {
  const int16_t n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return F("{\"buscando\":true}");
  if (n < 0) {
    WiFi.scanNetworks(true);
    return F("{\"buscando\":true}");
  }

  // De la más fuerte a la más débil, sin repetir nombres (un router con
  // varias antenas aparece una vez por cada una) y sin las ocultas.
  const uint8_t tope = n > 40 ? 40 : (uint8_t)n;
  uint8_t orden[40];
  for (uint8_t i = 0; i < tope; i++) orden[i] = i;
  for (uint8_t i = 0; i < tope; i++) {
    for (uint8_t j = i + 1; j < tope; j++) {
      if (WiFi.RSSI(orden[j]) > WiFi.RSSI(orden[i])) { const uint8_t t = orden[i]; orden[i] = orden[j]; orden[j] = t; }
    }
  }

  String s = F("{\"redes\":[");
  uint8_t puestas = 0;
  for (uint8_t i = 0; i < tope && puestas < 20; i++) {
    const String ssid = WiFi.SSID(orden[i]);
    if (ssid.length() == 0) continue;
    bool repetida = false;
    for (uint8_t j = 0; j < i && !repetida; j++) repetida = WiFi.SSID(orden[j]) == ssid;
    if (repetida) continue;
    if (puestas++) s += ',';
    s += F("{\"s\":\"");
    s += json(ssid);
    s += F("\",\"r\":");
    s += WiFi.RSSI(orden[i]);
    s += F(",\"c\":");
    s += WiFi.encryptionType(orden[i]) == WIFI_AUTH_OPEN ? '0' : '1';
    s += '}';
  }
  s += F("]}");
  WiFi.scanDelete();
  return s;
}

static void equipoJson(String& s, const char* id, const char* nombre, const String& ip, bool ocupado) {
  s += F("{\"id\":\"");     s += id;
  s += F("\",\"name\":\""); s += nombre;
  s += F("\",\"host\":\""); s += ip;
  s += F("\",\"busy\":");   s += ocupado ? '1' : '0';
  s += '}';
}

/**
 * Equipos compatibles que se ven desde acá, este incluido, en JSON.
 *
 * Quien entra por la red propia no llega a la red de casa, así que solo ve a
 * este equipo: los demás le darían direcciones a las que no tiene cómo ir.
 */
static String respuestaEquipos(bool porRedPropia) {
  ultimaMirada = millis();
  String s = F("{\"self\":\"");
  s += idEquipo;
  s += F("\",\"list\":[");
  equipoJson(s, idEquipo, nombreEquipo,
             porRedPropia ? WiFi.softAPIP().toString() : WiFi.localIP().toString(),
             wsAbierto || enlaceSerie);
  if (!porRedPropia) {
    xSemaphoreTake(candadoVecinos, portMAX_DELAY);
    for (uint8_t k = 0; k < nVecinos; k++) {
      s += ',';
      equipoJson(s, vecinos[k].id, vecinos[k].nombre, String(vecinos[k].ip), vecinos[k].ocupado);
    }
    xSemaphoreGive(candadoVecinos);
  }
  s += F("]}");
  return s;
}

// ── HTTP ───────────────────────────────────────────────────────────────────

/**
 * `abierto` deja leer la respuesta desde cualquier origen. Solo lo lleva
 * /peers: con la app abierta por http (desarrollo) la lista se pide directo,
 * sin ventana puente.
 */
static void responder(WiFiClient& c, const char* estado, const char* tipo, const String& cuerpo,
                      bool abierto) {
  c.print(F("HTTP/1.1 "));
  c.print(estado);
  c.print(F("\r\nContent-Type: "));
  c.print(tipo);
  if (abierto) c.print(F("\r\nAccess-Control-Allow-Origin: *"));
  c.print(F("\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "));
  c.print(cuerpo.length());
  c.print(F("\r\n\r\n"));
  c.print(cuerpo);
}

static void servirHtml(WiFiClient& c, const String& cuerpo) {
  responder(c, "200 OK", "text/html; charset=utf-8", cuerpo, false);
}

/** Manda al portal. Es lo que ven los chequeos de conexión de cada sistema. */
static void redirigir(WiFiClient& c, const char* ruta) {
  c.print(F("HTTP/1.1 302 Found\r\nLocation: http://"));
  c.print(WiFi.softAPIP().toString());
  c.print(ruta);
  c.print(F("\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: 0\r\n\r\n"));
}

/** Deshace el escapado de un formulario. */
static String urlDecode(const String& s) {
  String out;
  out.reserve(s.length());
  for (uint16_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < s.length()) {
      out += (char)strtoul(s.substring(i + 1, i + 3).c_str(), nullptr, 16);
      i += 2;
    } else {
      out += c;
    }
  }
  return out;
}

/** Valor de un campo del cuerpo de un formulario. */
static String campo(const String& cuerpo, const char* nombre) {
  const String marca = String(nombre) + "=";
  int i = cuerpo.startsWith(marca) ? 0 : cuerpo.indexOf(String("&") + marca);
  if (i < 0) return String();
  if (i > 0) i++;                          // saltar el & que precede a la marca
  i += marca.length();
  int fin = cuerpo.indexOf('&', i);
  if (fin < 0) fin = cuerpo.length();
  return urlDecode(cuerpo.substring(i, fin));
}

static void guardarRed(const String& ssid, const String& clave) {
  ajustes.begin("oryx", false);
  ajustes.putString("ssid", ssid);
  ajustes.putString("pass", clave);
  ajustes.end();
  nota("Red guardada: %s", ssid.length() ? ssid.c_str() : "(ninguna)");
}

/**
 * El pedido va a otro nombre que no es el del equipo.
 *
 * Solo pasa por la red propia, donde el DNS contesta todo con la dirección del
 * equipo: es un sistema que se fija si hay internet, o alguien que tipeó otra
 * dirección. En los dos casos lo que sirve es el portal.
 */
static bool esOtroSitio(const String& host) {
  if (host.length() == 0) return false;
  if (host == WiFi.softAPIP().toString()) return false;
  if (host == WiFi.localIP().toString()) return false;
  return !host.startsWith(MDNS_NOMBRE);
}

static void atenderHttp() {
  WiFiClient c = aceptar(servidorHttp);
  if (!c) return;

  const uint32_t t0 = millis();
  while (!c.available() && c.connected() && millis() - t0 < 1000) delay(1);

  // Primera línea: "GET /ruta?consulta HTTP/1.1".
  String pedido = c.readStringUntil('\n');
  pedido.trim();

  // Cabeceras: interesan el nombre al que se pidió y cuánto mide el cuerpo.
  String host;
  int largoCuerpo = 0;
  while (millis() - t0 < 2000) {
    if (!c.available()) { if (!c.connected()) break; delay(1); continue; }
    String h = c.readStringUntil('\n');
    h.trim();
    if (h.length() == 0) break;
    String baja = h;
    baja.toLowerCase();
    if (baja.startsWith("content-length:")) {
      largoCuerpo = h.substring(h.indexOf(':') + 1).toInt();
    } else if (baja.startsWith("host:")) {
      host = baja.substring(5);
      host.trim();
      const int dosPuntos = host.indexOf(':');
      if (dosPuntos >= 0) host = host.substring(0, dosPuntos);
    }
  }

  String cuerpo;
  if (largoCuerpo > 0 && largoCuerpo < 1024) {
    while ((int)cuerpo.length() < largoCuerpo && millis() - t0 < 3000) {
      if (c.available()) cuerpo += (char)c.read();
      else delay(1);
    }
  }

  const int e1 = pedido.indexOf(' ');
  const int e2 = pedido.indexOf(' ', e1 + 1);
  const String metodo = e1 > 0 ? pedido.substring(0, e1) : String();
  String ruta = (e1 > 0 && e2 > e1) ? pedido.substring(e1 + 1, e2) : String("/");
  const int q = ruta.indexOf('?');
  if (q >= 0) ruta = ruta.substring(0, q);

  const bool porRedPropia = c.localIP() == WiFi.softAPIP();

  bool reiniciar = false;
  if (porRedPropia && esOtroSitio(host)) {
    redirigir(c, "/wifi");
  } else if (metodo == "POST" && ruta == "/wifi") {
    const String ssid  = campo(cuerpo, "s");
    const String clave = campo(cuerpo, "p");
    String nombre = ssid;
    nombre.trim();
    if (nombre.length() == 0 || ssid.length() > 32) {
      servirHtml(c, paginaWifi("Elige una red de la lista o escribe su nombre."));
    } else if (clave.length() > 0 && (clave.length() < 8 || clave.length() > 63)) {
      servirHtml(c, paginaWifi("La clave de una red WiFi tiene entre 8 y 63 caracteres."));
    } else {
      guardarRed(ssid, clave);
      servirHtml(c, paginaGuardado(ssid));
      reiniciar = true;
    }
  } else if (metodo == "POST" && ruta == "/olvidar") {
    guardarRed("", "");
    servirHtml(c, paginaOlvidada());
    reiniciar = true;
  } else if (ruta == "/bridge") {
    servirHtml(c, paginaPuente());
  } else if (ruta == "/peers") {
    responder(c, "200 OK", "application/json", respuestaEquipos(porRedPropia), true);
  } else if (ruta == "/scan") {
    responder(c, "200 OK", "application/json", respuestaBusqueda(), false);
  } else if (ruta == "/wifi") {
    servirHtml(c, paginaWifi(nullptr));
  } else if (ruta == "/" || ruta == "/index.html") {
    servirHtml(c, paginaInicio());
  } else if (porRedPropia) {
    // generate_204, hotspot-detect.html, connecttest.txt… van todos al portal
    redirigir(c, "/wifi");
  } else {
    responder(c, "404 Not Found", "text/plain; charset=utf-8", String(), false);
  }

  delay(1);
  c.stop();

  // Reiniciar es la forma más limpia de levantar en la red nueva: no queda
  // ningún estado a medio armar que haya que reordenar a mano. La espera es
  // para que la página de despedida termine de salir antes.
  if (reiniciar) { delay(600); ESP.restart(); }
}

// ── WebSocket ──────────────────────────────────────────────────────────────

static void cerrarWs() {
  if (!wsAbierto) return;
  clienteWs.stop();
  wsAbierto = false;
  rxLargo = 0;
  cargando = false;
  // Si se cae el enlace la señal se para: nadie quedaría para pararla.
  detenerYEsperar();
  salidasIdle();
  nota("Enlace WiFi cerrado");
}

static void atenderWs() {
  if (!wsAbierto) {
    WiFiClient nuevo = aceptar(servidorWs);
    if (!nuevo) return;
    if (wsHandshake(nuevo)) {
      clienteWs = nuevo;
      wsAbierto = true;
      rxLargo = 0;
      nota("Enlace WiFi abierto");
      mandarSaludo();
      mandarEstado();
    } else {
      nuevo.stop();
    }
    return;
  }

  for (uint8_t vueltas = 0; vueltas < 16; vueltas++) {
    const WsRead r = wsReadText(clienteWs, rx, sizeof(rx), rxLargo);
    if (r == WS_CERRAR) { cerrarWs(); return; }
    if (r == WS_NADA)   return;
    procesarMensaje(rx);
    rxLargo = 0;
  }
}

static void tareaRed(void*) {
  uint32_t ultimoEstado = 0;
  for (;;) {
    // El puerto serie se atiende dos veces por vuelta: es el único enlace sin
    // control de flujo, así que su buffer no puede esperar a que termine todo
    // lo demás.
    atenderSerie();
    dns.processNextRequest();
    atenderHttp();
    atenderSerie();
    atenderWs();
    vigilarRed();
    anunciarOcupado(wsAbierto || enlaceSerie);
    if ((wsAbierto || enlaceSerie) && millis() - ultimoEstado > 250) {
      ultimoEstado = millis();
      mandarEstado();
    }
    vTaskDelay(1);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ARRANQUE
// ═══════════════════════════════════════════════════════════════════════════

/** Se une a la red guardada, si hay una. Nunca apaga la red propia. */
static void unirseARedGuardada() {
  ajustes.begin("oryx", true);
  staSsid  = ajustes.getString("ssid", "");
  staClave = ajustes.getString("pass", "");
  ajustes.end();
  if (staSsid.length() == 0) {
    faseRed = RED_SIN_CONFIGURAR;
    nota("Sin red de casa configurada");
    return;
  }

  // Se espera acá mismo para que el saludo del monitor serie ya diga en qué
  // dirección quedó. Lo que pase después lo sigue vigilarRed().
  nota("Uniéndose a %s…", staSsid.c_str());
  entrarARed();
  while (WiFi.status() != WL_CONNECTED && millis() - faseDesde < PLAZO_RED_MS) delay(200);
  vigilarRed();
}

void setup() {
  Serial.setRxBufferSize(RX_SERIE);
  Serial.begin(BAUDIOS);
  delay(200);

  armarTablaGpio();
  for (uint8_t i = 0; i < MAX_CANALES_D; i++) {
    pinMode(PINES[i], OUTPUT);
    digitalWrite(PINES[i], LOW);
  }
  recalcularFactor();
  escribirReferencia();

  // Nombre propio con los dos últimos bytes de la MAC: no cambia nunca y no
  // se repite entre equipos, que es lo que hace falta para tener varios juntos.
  const uint64_t mac = ESP.getEfuseMac();   // el primer byte de la MAC va en los bits bajos
  snprintf(idEquipo, sizeof(idEquipo), "%02X%02X",
           (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  snprintf(nombreEquipo, sizeof(nombreEquipo), "%s-%s", PREFIJO_NOMBRE, idEquipo);
  char nombreRouter[24];
  snprintf(nombreRouter, sizeof(nombreRouter), "%s-%s", MDNS_NOMBRE, idEquipo);
  for (char* p = nombreRouter; *p; p++) *p = tolower(*p);

  /* La red propia se levanta siempre, esté o no configurada la de casa: es el
     portal de configuración y es la salida cuando no hay router a mano. */
  WiFi.setHostname(nombreRouter);         // así aparece en la lista del router
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(nombreEquipo, AP_CLAVE);
  // Sin ahorro de energía: con él la latencia del enlace sube a cientos de ms
  // y la app se queda sin estado mientras la señal corre.
  WiFi.setSleep(false);
  unirseARedGuardada();

  // Todos comparten oryx-signal.local como puerta de entrada; lo que los
  // distingue es el servicio _oryx._tcp, con su nombre y su identificador.
  if (MDNS.begin(MDNS_NOMBRE)) {
    MDNS.setInstanceName(nombreEquipo);
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("oryx", "tcp", 81);
    char proto[4];
    snprintf(proto, sizeof(proto), "%u", (unsigned)PROTO);
    mdns_service_txt_item_set(MDNS_SERVICIO, MDNS_PROTO, "id", idEquipo);
    mdns_service_txt_item_set(MDNS_SERVICIO, MDNS_PROTO, "proto", proto);
    mdns_service_txt_item_set(MDNS_SERVICIO, MDNS_PROTO, "busy", "0");
  }

  // Cualquier nombre que se pida desde la red propia apunta al equipo: así el
  // teléfono o la computadora abren el portal solos al conectarse.
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());

  servidorHttp.begin();
  servidorHttp.setNoDelay(true);
  servidorWs.begin();
  servidorWs.setNoDelay(true);

  nota("");
  nota("Oryx Signal · generador ESP32");
  nota("Equipo    : %s", nombreEquipo);
  nota("USB       : este mismo puerto, a %u baudios", BAUDIOS);
  nota("Red propia: %s / %s", nombreEquipo, AP_CLAVE);
  nota("Portal    : http://%s/", WiFi.softAPIP().toString().c_str());
  if (faseRed == RED_ADENTRO) {
    nota("En tu red : http://%s/  ·  http://%s.local/",
         WiFi.localIP().toString().c_str(), MDNS_NOMBRE);
  }

  /* El bucle de espera fina no cede el procesador, que es exactamente lo que
     el vigilante del núcleo 1 considera un cuelgue. Se apaga a propósito: la
     red vive en el otro núcleo y sigue vigilada. */
  disableCore1WDT();

  candadoVecinos = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(tareaRed,          "red",  8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(tareaVecinos,  "vecinos",  4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(tareaReproduccion, "play", 4096, nullptr, 5, nullptr, 1);
}

void loop() {
  vTaskDelay(1000);
}
