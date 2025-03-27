const int pinInicio = 2; // PD2 - PD7 en ATmega328P corresponden a pines digitales 2-7 en Arduino
const char* botones[] = {"Arriba", "Abajo", "Izquierda", "Derecha", "Accion A", "Accion B"};
int estadoAnterior = 0;

void setup() {
	Serial.begin(115200);
	for (int i = 0; i < 6; i++) {
		pinMode(pinInicio + i, INPUT_PULLUP); // Configurar pines como entrada con pull-up
	}
	estadoAnterior = leerEstado();
}

void loop() {
	int estadoActual = leerEstado();
	int cambio = (estadoAnterior ^ estadoActual) & ~estadoActual; // Detectar flanco descendente
	
	for (int i = 0; i < 6; i++) {
		if (cambio & (1 << i)) {
			Serial.println(botones[i]);
		}
	}
	
	estadoAnterior = estadoActual;
	delay(50); // Pequeño debounce
}

int leerEstado() {
	int estado = 0;
	for (int i = 0; i < 6; i++) {
		if (digitalRead(pinInicio + i) == LOW) {
			estado |= (1 << i);
		}
	}
	return estado;
}
