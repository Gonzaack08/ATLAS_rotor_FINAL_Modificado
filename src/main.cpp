#include <Arduino.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <math.h>

// wifi para conectar gpredict
const char* WIFI_SSID = "TU_WIFI";
const char* WIFI_PASS = "TU_CLAVE";
constexpr uint16_t PUERTO_GPRE = 5000;

// Pines que usamos en la placa.
constexpr uint8_t AZ_STEP = 23;
constexpr uint8_t AZ_DIR  = 22;
constexpr uint8_t EL_STEP = 19;
constexpr uint8_t EL_DIR  = 18;
constexpr uint8_t AZ_MIN  = 32;
constexpr uint8_t AZ_MAX  = 33;
constexpr uint8_t EL_MIN  = 13;
constexpr uint8_t EL_MAX  = 14;

// Cambiar a true si algun motor gira al reves.
constexpr bool INVERTIR_DIR_AZ = false;
constexpr bool INVERTIR_DIR_EL = false;

// Relacion 14:40 y microstepping configurado en 1/4.
constexpr int PASOS_POR_VUELTA_MOTOR = 200;
constexpr int MICROSTEPPING = 4;
constexpr float DIENTES_MOTOR = 14.0f;
constexpr float DIENTES_EJE = 40.0f;
constexpr float RELACION = DIENTES_EJE / DIENTES_MOTOR;
constexpr float PASOS_POR_VUELTA_EJE =
    PASOS_POR_VUELTA_MOTOR * MICROSTEPPING * RELACION;
constexpr float RANGO_AZ_GRADOS = 180.0f;
constexpr float RANGO_EL_GRADOS = 90.0f;

constexpr float VEL_AZ = 200.0f;
constexpr float VEL_EL = 160.0f;
constexpr float ACEL_AZ = 100.0f;
constexpr float ACEL_EL = 80.0f;
constexpr float VEL_HOME = 110.0f;
constexpr float VEL_HOME_FINA = 25.0f;
constexpr float ACEL_HOME = 90.0f;
constexpr float GRADOS_DE_RETIRO = 5.0f;
constexpr uint32_t TIMEOUT_HOME_MS = 90000UL;

AccelStepper motorAZ(AccelStepper::DRIVER, AZ_STEP, AZ_DIR);
AccelStepper motorEL(AccelStepper::DRIVER, EL_STEP, EL_DIR);
WiFiServer servidor(PUERTO_GPRE);
WiFiClient cliente;
bool servidorListo = false;
String entradaTCP, entradaSerie;

enum EstadoHome : uint8_t {
  SIN_INICIAR,
  BUSCAR_MIN,
  RETIRARSE_DE_MIN,
  BUSCAR_MIN_LENTO,
  EJE_LISTO,
  EJE_ERROR
};

struct Eje {
  const char* nombre;
  AccelStepper* motor;
  uint8_t minPin;
  uint8_t maxPin;
  float rangoGrados;
  float velocidad;
  float aceleracion;
  long pasosRango;
  long pasosRetiro;
  long pasosBusqueda;
  EstadoHome home;
  uint32_t inicioFase;//PENE BIEN GRANDE UwU oni-chann
  long posicionInicioFase;
  bool bloqueoMinNotificado;
  bool bloqueoMaxNotificado;
};

Eje az = {"AZ", &motorAZ, AZ_MIN, AZ_MAX, RANGO_AZ_GRADOS,
          VEL_AZ, ACEL_AZ, 0, 0, 0, SIN_INICIAR, 0, 0, false, false};
Eje el = {"EL", &motorEL, EL_MIN, EL_MAX, RANGO_EL_GRADOS,
          VEL_EL, ACEL_EL, 0, 0, 0, SIN_INICIAR, 0, 0, false, false};

bool errorGeneral = false;

// Finales NC: HIGH indica que se abrio el contacto.
bool minActivo(const Eje& e) { return digitalRead(e.minPin) == HIGH; }
bool maxActivo(const Eje& e) { return digitalRead(e.maxPin) == HIGH; }
bool homingCompleto() {
  return !errorGeneral && az.home == EJE_LISTO && el.home == EJE_LISTO;
}

void detener(Eje& e) {
  e.motor->setCurrentPosition(e.motor->currentPosition());
}

void errorSeguridad(const char* razon) {
  detener(az);
  detener(el);
  errorGeneral = true;
  az.home = EJE_ERROR;
  el.home = EJE_ERROR;
  Serial.print("ERROR ATLAS: ");
  Serial.println(razon);
  Serial.println("Revisar finales/sentidos/cableado. Enviar H para re-homing.");
}

void iniciarFase(Eje& e, EstadoHome fase) {
  e.home = fase;
  e.inicioFase = millis();
  e.posicionInicioFase = e.motor->currentPosition();
  AccelStepper& m = *e.motor;
  m.setAcceleration(ACEL_HOME);
  m.setMaxSpeed(VEL_HOME);

  switch (fase) {
    case BUSCAR_MIN:
      m.moveTo(m.currentPosition() - e.pasosBusqueda);
      break;
    case RETIRARSE_DE_MIN:
      m.moveTo(m.currentPosition() + e.pasosRetiro);
      break;
    case BUSCAR_MIN_LENTO:
      m.setMaxSpeed(VEL_HOME_FINA);
      m.moveTo(m.currentPosition() - 2L * e.pasosRetiro);
      break;
    case EJE_LISTO:
      m.setCurrentPosition(0);
      m.setMaxSpeed(e.velocidad);
      m.setAcceleration(e.aceleracion);
      Serial.print(e.nombre);
      Serial.println(" homing finalizado. CERO = 0 grados.");
      break;
    default: break;
  }
}

// Primero hace cero de AZ y despues de EL.
void empezarHoming() {
  detener(az);
  detener(el);
  errorGeneral = false;
  az.home = SIN_INICIAR;
  el.home = SIN_INICIAR;
  az.bloqueoMinNotificado = az.bloqueoMaxNotificado = false;
  el.bloqueoMinNotificado = el.bloqueoMaxNotificado = false;

  if ((minActivo(az) && maxActivo(az)) ||
      (minActivo(el) && maxActivo(el))) {
    errorSeguridad("MIN y MAX activos simultaneamente en un eje");
    return;
  }
  Serial.println("ATLAS: homing de AZ y despues EL (solo finales MIN)");
  iniciarFase(az, minActivo(az) ? RETIRARSE_DE_MIN : BUSCAR_MIN);
}

void ejecutarHoming(Eje& e) {
  if (errorGeneral || e.home == SIN_INICIAR ||
      e.home == EJE_LISTO || e.home == EJE_ERROR) return;

  if (minActivo(e) && maxActivo(e)) {
    errorSeguridad("Dos finales de carrera activos a la vez");
    return;
  }
  if ((uint32_t)(millis() - e.inicioFase) > TIMEOUT_HOME_MS) {
    errorSeguridad("Tiempo de homing excedido");
    return;
  }

  AccelStepper& m = *e.motor;
  bool llego = (m.distanceToGo() == 0);

  switch (e.home) {
    case BUSCAR_MIN:
      if (minActivo(e)) {
        detener(e);
        iniciarFase(e, RETIRARSE_DE_MIN);
      } else if (maxActivo(e) &&
                 e.posicionInicioFase - m.currentPosition() > e.pasosRetiro) {
        errorSeguridad("MAX no se libero al buscar MIN: verificar direccion");
      } else if (llego) {
        errorSeguridad("No se encontro MIN en el recorrido de busqueda");
      } else {
        m.run();
      }
      break;

    case RETIRARSE_DE_MIN:
      if (maxActivo(e)) {
        errorSeguridad("MAX activo durante retiro de MIN");
      } else if (llego) {
        if (minActivo(e)) errorSeguridad("MIN no se libera: contacto/cable abierto");
        else iniciarFase(e, BUSCAR_MIN_LENTO);
      } else {
        m.run();
        if (maxActivo(e)) errorSeguridad("MAX alcanzado durante retiro de MIN");
      }
      break;

    case BUSCAR_MIN_LENTO:
      if (minActivo(e)) {
        detener(e);
        iniciarFase(e, EJE_LISTO);
      } else if (llego) {
        errorSeguridad("No se encontro MIN en aproximacion lenta");
      } else {
        m.run();
      }
      break;

    default: break;
  }
}

// En seguimiento, deja salir del tope pero no avanzar contra el.
void verificarLimitesYSeguir(Eje& e) {
  if (errorGeneral || e.home != EJE_LISTO) return;
  AccelStepper& m = *e.motor;

  bool enMin = minActivo(e);
  bool enMax = maxActivo(e);
  if (enMin && enMax) {
    errorSeguridad("MIN y MAX activos a la vez durante seguimiento");
    return;
  }

  long pendiente = m.distanceToGo();
  float velocidadInstantanea = m.speed();

  if (enMin && (velocidadInstantanea < 0.0f ||
                (velocidadInstantanea == 0.0f && pendiente < 0))) {
    detener(e);
    if (!e.bloqueoMinNotificado) {
      Serial.print(e.nombre);
      Serial.println(": limite INICIO detectado, giro negativo bloqueado");
      e.bloqueoMinNotificado = true;
    }
    return;
  }
  if (enMax && (velocidadInstantanea > 0.0f ||
                (velocidadInstantanea == 0.0f && pendiente > 0))) {
    detener(e);
    if (!e.bloqueoMaxNotificado) {
      Serial.print(e.nombre);
      Serial.println(": limite MAX detectado, giro positivo bloqueado");
      e.bloqueoMaxNotificado = true;
    }
    return;
  }

  if (!enMin) e.bloqueoMinNotificado = false;
  if (!enMax) e.bloqueoMaxNotificado = false;

  long destino = m.targetPosition();
  if (destino < 0) m.moveTo(0);
  if (destino > e.pasosRango) m.moveTo(e.pasosRango);

  m.run();

  if ((minActivo(e) && m.speed() < 0.0f) ||
      (maxActivo(e) && m.speed() > 0.0f)) {
    detener(e);
    Serial.print(e.nombre);
    Serial.println(": movimiento detenido por final de carrera");
  }
}

float gradosActuales(const Eje& e) {
  float grados = (float)e.motor->currentPosition() *
                 e.rangoGrados / (float)e.pasosRango;
  return constrain(grados, 0.0f, e.rangoGrados);
}

long gradosAPasos(const Eje& e, float grados) {
  return lroundf(grados * (float)e.pasosRango / e.rangoGrados);
}

void respuesta(Stream& canal, int codigo) {
  canal.print("RPRT ");
  canal.println(codigo);
}

// Comandos de Gpredict: P mueve, p consulta posicion.
void procesarComando(String linea, Stream& canal) {
  linea.trim();
  if (!linea.length()) return;

  if (linea == "p" || linea == "\\get_pos") {
    if (!homingCompleto()) respuesta(canal, -9);
    else {
      canal.println(gradosActuales(az), 6);
      canal.println(gradosActuales(el), 6);
    }
    return;
  }

  if (linea.startsWith("P ") || linea.startsWith("\\set_pos ")) {
    float azDest, elDest;
    char adicional;
    int campos = linea.startsWith("P ")
      ? sscanf(linea.c_str(), "P %f %f %c", &azDest, &elDest, &adicional)
      : sscanf(linea.c_str(), "\\set_pos %f %f %c", &azDest, &elDest, &adicional);

    if (campos != 2 || !isfinite(azDest) || !isfinite(elDest) ||
        azDest < 0.0f || azDest > RANGO_AZ_GRADOS ||
        elDest < 0.0f || elDest > RANGO_EL_GRADOS) {
      respuesta(canal, -1);
    } else if (!homingCompleto()) {
      respuesta(canal, -9);
    } else {
      az.motor->moveTo(gradosAPasos(az, azDest));
      el.motor->moveTo(gradosAPasos(el, elDest));
      respuesta(canal, 0);
    }
    return;
  }

  if (linea == "S" || linea == "\\stop") {
    if (!homingCompleto()) errorSeguridad("Homing detenido por orden STOP");
    else {
      detener(az);
      detener(el);
    }
    respuesta(canal, 0);
    return;
  }

  if (linea == "K" || linea == "\\park") {
    if (!homingCompleto()) respuesta(canal, -9);
    else {
      az.motor->moveTo(0);
      el.motor->moveTo(0);
      respuesta(canal, 0);
    }
    return;
  }

  if (linea == "H") {
    empezarHoming();
    respuesta(canal, errorGeneral ? -9 : 0);
    return;
  }

  respuesta(canal, -4);
}

void leerSerie() {
  uint8_t cantidad = 0;
  while (Serial.available() && cantidad++ < 48) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      procesarComando(entradaSerie, Serial);
      entradaSerie = "";
    } else if (ch != '\r') {
      if (entradaSerie.length() < 80) entradaSerie += ch;
      else entradaSerie = "";
    }
  }
}

void leerTCP() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!servidorListo) {
    servidor.begin();
    servidor.setNoDelay(true);
    servidorListo = true;
    Serial.print("Gpredict IP ");
    Serial.print(WiFi.localIP());
    Serial.print(" puerto ");
    Serial.println(PUERTO_GPRE);
  }

  if (!cliente || !cliente.connected()) {
    cliente.stop();
    WiFiClient nuevo = servidor.available();
    if (nuevo) {
      cliente = nuevo;
      entradaTCP = "";
      Serial.println("Cliente Gpredict conectado");
    }
  }

  if (cliente && cliente.connected()) {
    uint8_t cantidad = 0;
    while (cliente.available() && cantidad++ < 48) {
      char ch = (char)cliente.read();
      if (ch == '\n') {
        procesarComando(entradaTCP, cliente);
        entradaTCP = "";
      } else if (ch != '\r') {
        if (entradaTCP.length() < 80) entradaTCP += ch;
        else entradaTCP = "";
      }
    }
  }
}

void prepararEje(Eje& e) {
  e.pasosRango = lroundf(PASOS_POR_VUELTA_EJE *
                         e.rangoGrados / 360.0f);
  e.pasosRetiro = max(20L, lroundf(PASOS_POR_VUELTA_EJE *
                                    GRADOS_DE_RETIRO / 360.0f));
  e.pasosBusqueda = lroundf(e.pasosRango * 1.5f) + e.pasosRetiro;
  e.motor->setMinPulseWidth(8);
  e.motor->setMaxSpeed(e.velocidad);
  e.motor->setAcceleration(e.aceleracion);
}

void setup() {
  Serial.begin(115200);

  pinMode(AZ_MIN, INPUT_PULLUP);
  pinMode(AZ_MAX, INPUT_PULLUP);
  pinMode(EL_MIN, INPUT_PULLUP);
  pinMode(EL_MAX, INPUT_PULLUP);

  motorAZ.setPinsInverted(INVERTIR_DIR_AZ, false, false);
  motorEL.setPinsInverted(INVERTIR_DIR_EL, false, false);
  prepararEje(az);
  prepararEje(el);

  Serial.println("ATLAS v2 | Homing MIN + limites permanentes + Gpredict");
  Serial.print("AZ 0-180, micro-pasos: "); Serial.println(az.pasosRango);
  Serial.print("EL 0-90, micro-pasos: "); Serial.println(el.pasosRango);
  Serial.println("Finales NC: LOW libre, HIGH pulsado/cable abierto");
  Serial.println("Serie: H (homing), P 90 45 (destino), p (posicion), S (stop)");

  if (strcmp(WIFI_SSID, "TU_WIFI") != 0 && WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  empezarHoming();
}

void loop() {
  leerSerie();
  leerTCP();

  if (errorGeneral) return;
  if (az.home != EJE_LISTO) {
    ejecutarHoming(az);
  } else if (el.home != EJE_LISTO) {
    if (el.home == SIN_INICIAR)
      iniciarFase(el, minActivo(el) ? RETIRARSE_DE_MIN : BUSCAR_MIN);
    ejecutarHoming(el);
  } else {
    verificarLimitesYSeguir(az);
    verificarLimitesYSeguir(el);
  }
}
