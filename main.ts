//  Limite pour éviter l'intégration excessive
function constrain(value: number, min_val: number, max_val: number): number {
    //  Contraint une valeur dans une plage donnée.
    return Math.max(Math.min(value, max_val), min_val)
}

let right_speed = 0
let left_speed = 0
let correction = 0
let derivative = 0
let integral = 0
let previous_error = 0
let error = 0
let input_buton = false
datalogger.setColumnTitles("left", "right")
// Attention modifier la vitesse de base avec prudence maximum 40 (a partire de 30 l'éfficacité de la correction n'est plus assuré)
let base_speed = 30
let Kp = 110
let Ki = 9
let Kd = 40
// MAX_SPEED peut etre augmenté de manière significative mais pas trop non plus pour évité
//  Les wind-up (fait que l'inégrale augmente trop, trop rapidement)
let MAX_SPEED = 40
let INTEGRAL_LIMIT = 20
//  Temps limite en millisecondes (15 secondes)
let TIME_LIMIT = 15000
let start_time = control.millis()
//  Chronomètre initialisé au démarrage
//  Distance minimale pour détecter un obstacle en centimètres
let MIN_DISTANCE = 5
basic.forever(function on_forever() {
    let Kp_temps: number;
    let input_buton2: boolean;
    let MIN_SPEED: number;
    let distance: number;
    
    //  Si le bouton A est préssé, input_buton passe a True
    //  et permet au programme présent dans la boucle
    //  while de s'activer
    // Ici le code de modification du PID en fonction du temps:
    if (control.millis() - start_time <= 10000) {
        Kp_temps = Kp / 2
    } else {
        Kp_temps = Kp
    }
    
    if (input.buttonIsPressed(Button.A) == true) {
        start_time = control.millis()
        input_buton2 = true
    }
    
    if (input.buttonIsPressed(Button.A) == true && input.buttonIsPressed(Button.B) == true) {
        control.reset()
    }
    
    while (input_buton2 == true) {
        MIN_SPEED = 0
        //  Chronomètre : Arrêter le robot au bout du temps limite
        //  control.millis() - start_time équivaut au temps actuel
        if (control.millis() - start_time >= TIME_LIMIT) {
            maqueen.motorStop(maqueen.Motors.All)
            basic.showIcon(IconNames.Square)
            return
        }
        
        //  Arrête l'exécution de la fonction
        //  Détection des obstacles avec le capteur à ultrasons
        distance = maqueen.Ultrasonic()
        if (distance < MIN_DISTANCE) {
            maqueen.motorStop(maqueen.Motors.All)
            return
        }
        
        //  Arrête l'exécution de la fonction pour éviter les collisions
        //  Détection des capteurs de ligne
        if (maqueen.readPatrol(maqueen.Patrol.PatrolLeft) == 1 && maqueen.readPatrol(maqueen.Patrol.PatrolRight) == 0) {
            error = -1
        } else if (maqueen.readPatrol(maqueen.Patrol.PatrolLeft) == 0 && maqueen.readPatrol(maqueen.Patrol.PatrolRight) == 1) {
            error = 1
        } else if (maqueen.readPatrol(maqueen.Patrol.PatrolLeft) == 1 && maqueen.readPatrol(maqueen.Patrol.PatrolRight) == 1) {
            error = 0
        } else {
            error = previous_error
        }
        
        //  Maintient de l'erreur précédente si aucune ligne détectée
        //  PID
        integral += error
        integral = constrain(integral, 0 - INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        //  Anti-windup
        derivative = error - previous_error
        correction = constrain(Kp * error + Ki * integral + Kd * derivative, 0 - MAX_SPEED, MAX_SPEED)
        //  Calcul des vitesses
        left_speed = constrain(base_speed - correction, MIN_SPEED, MAX_SPEED)
        right_speed = constrain(base_speed + correction, MIN_SPEED, MAX_SPEED)
        //  Envoi aux moteurs
        if (left_speed >= 0) {
            maqueen.motorRun(maqueen.Motors.M1, maqueen.Dir.CW, left_speed)
        } else {
            maqueen.motorRun(maqueen.Motors.M1, maqueen.Dir.CCW, Math.abs(left_speed))
        }
        
        if (right_speed >= 0) {
            maqueen.motorRun(maqueen.Motors.M2, maqueen.Dir.CW, right_speed)
        } else {
            maqueen.motorRun(maqueen.Motors.M2, maqueen.Dir.CCW, Math.abs(right_speed))
        }
        
        //  Enregistrement des données
        datalogger.log(datalogger.createCV("left", left_speed))
        datalogger.log(datalogger.createCV("right", right_speed))
        //  Mise à jour de l'erreur précédente
        previous_error = error
    }
})
