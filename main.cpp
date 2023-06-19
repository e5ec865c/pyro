/*
SOLENOID CONTROL PROGRAM FOR THE HEART MACHINE, BRC

N solenoids controlled by BPM pot, a duty-cycle pot, a phase pot, a
switch and four buttons.  The buttons always act as manual opens for
the valves.  The switch enables and disables parametric programmed
control that can also open the valves.  Manual and program control can
happen at the same time.
*/


// the macro sets or clears the appropriate bit in port D if the pin
// is less than 8 or port B if between 8 and 13
#define fastWrite(_pin_, _state_) ( _pin_ < 8 ? (_state_ ?  PORTD |= 1 << _pin_ : PORTD &= ~(1 << _pin_ )) : (_state_ ?  PORTB |= 1 << (_pin_ -8) : PORTB &= ~(1 << (_pin_ -8)  )))


int numValves = 4;

// digital input wiring
int runSwitchPin = 7;
int manualButtonPin[4] = { 6, 8, 12, 13 };

// analog input wiring
int dutyPotPin = 0;
int bpmPotPin = 1;
int phasePotPin = 2;

// output wiring
int valvePin[4] = { 2, 3, 4, 5 };
int ledPin = 13; // onboard, fixed


// Note that duty-cycle isn't constrained to min timing like BPM is.  
// The idea is that there could be some glitch-art fun at the edge of what
// the solenoids can do.
#if 0
int minBpm = 60;
int maxBpm = 1200; 
#else
// for LED-based testing
int minBpm = 30;
int maxBpm = 240; 
#endif

// technical constants...
// On an Duemillanove the main loop takes ~1ms, let's conserve
// some power by waiting just a little bit between samples.
int samplingInterval = 2; // ms
float msPerSec = 60000.0;

struct State
{
  State()
  {
    t = 0;
    phase = 0.0;
    period = 0.0;
    duty = 0.0;
  }
  
  long t;
  float phase; // 0-1
  float period;
  float duty;
};

// the key primitive
struct SquareWave
{
  SquareWave()
  {}

  bool sample( long t )
  {
    now.t = t;
    
    float offset = then.phase * then.period;
    
    t = t - offset;

    long r = t % (long)(then.period);

    bool on = r < then.duty;

    if( t > then.t + then.period )
    {
      then = now;
    }
    
    return on;
  }

  State then;
  State now;
};


SquareWave signal[4];


void setup()
{
#if 0 // enable for serial debugging on actual hardware
  Serial.begin( 9600 );
  Serial.flush();
#endif

  pinMode( bpmPotPin, INPUT );
  pinMode( dutyPotPin, INPUT );
  pinMode( phasePotPin, INPUT );

  pinMode( runSwitchPin, INPUT_PULLUP );
  for( int v = 0; v < numValves; ++v )
    pinMode( manualButtonPin[v], INPUT_PULLUP );

  pinMode( ledPin, OUTPUT );
  for( int v = 0; v < numValves; ++v )
    pinMode( valvePin[v], OUTPUT );
}


void loop ()
{
  long time = millis();

  bool fire[4];

#if 1
  int run = digitalRead( runSwitchPin );

  for( int v = 0; v < numValves; ++v )
    fire[v] = digitalRead( manualButtonPin[v] );
#else
  // for testing without a run switch
  int run = 1;
  fire[0] = false;
  fire[1] = false;
  fire[2] = false;
  fire[3] = false;
#endif

  if( run )
  {
    // Read normalized pot values (0.0 to 1.0).
    float bpmParam = (1023.0 - analogRead(bpmPotPin)) / 1023.0;
    float dutyParam = (1023.0 - analogRead(dutyPotPin)) / 1023.0;
    float phaseParam = (1023.0 - analogRead(phasePotPin)) / 1023.0;

    // Give the BPM pot a flatter response at the low end where
    // the differences are danceable.  The same for phase, where
    // smaller differences are more noticable at the low end.
    // At high duty cycle all patterns merge into one big whoosh,
    // So let's compress fine control over that too.
    // As a first take, just square them all...
    bpmParam = bpmParam * bpmParam;
    phaseParam = phaseParam * phaseParam;
    dutyParam = dutyParam * dutyParam;

    float bpm = minBpm + ( (maxBpm-minBpm) * bpmParam );

    // In milliseconds...
    float period = msPerSec / bpm;
    float duty = period * dutyParam;

    for( int v = 0; v < numValves; ++v )
    {
      float fv = float(v) / float(numValves);

      signal[v].now.period = period;
      signal[v].now.duty = duty;
      signal[v].now.phase = phaseParam * fv;

      fire[v] = fire[v] || signal[v].sample( time );
    }
  }

  for( int v = 0; v < numValves; ++v )
  {
    if( fire[v] )
      fastWrite( valvePin[v], HIGH );
    else
      fastWrite( valvePin[v], LOW );
  }

  if( fire[0] )
    fastWrite( ledPin, HIGH );
  else
    fastWrite( ledPin, LOW );

  delay( samplingInterval );
}
