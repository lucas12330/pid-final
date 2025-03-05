# Limite pour éviter l'intégration excessive
def constrain(value: number, min_val: number, max_val: number):
    # Contraint une valeur dans une plage donnée.
    return max(min(value, max_val), min_val)

right_speed = 0
left_speed = 0
correction = 0
derivative = 0
integral = 0
previous_error = 0
error = 0
input_buton = False
datalogger.set_column_titles("left", "right")

#Attention modifier la vitesse de base avec prudence maximum 40 (a partire de 30 l'éfficacité de la correction n'est plus assuré)
base_speed = 30
Kp = 110
Ki = 9
Kd = 40

#MAX_SPEED peut etre augmenté de manière significative mais pas trop non plus pour évité
# Les wind-up (fait que l'inégrale augmente trop, trop rapidement)
MAX_SPEED = 40
INTEGRAL_LIMIT = 20
# Temps limite en millisecondes (15 secondes)
TIME_LIMIT = 15000
start_time = control.millis()
# Chronomètre initialisé au démarrage
# Distance minimale pour détecter un obstacle en centimètres
MIN_DISTANCE = 5

def on_forever():
    global start_time, error, integral, derivative, correction, left_speed, right_speed, previous_error
    # Si le bouton A est préssé, input_buton passe a True
    # et permet au programme présent dans la boucle
    # while de s'activer

    #Ici le code de modification du PID en fonction du temps:
    if control.millis() - start_time <= 10000:
        Kp_temps = Kp/2

    else:
        Kp_temps = Kp

    
    if input.button_is_pressed(Button.A) == True:
        start_time = control.millis()
        input_buton2 = True
    if input.button_is_pressed(Button.A) == True and input.button_is_pressed(Button.B) == True:
        control.reset()
    while input_buton2 == True:
        MIN_SPEED = 0
        # Chronomètre : Arrêter le robot au bout du temps limite
        # control.millis() - start_time équivaut au temps actuel
        if control.millis() - start_time >= TIME_LIMIT:
            maqueen.motor_stop(maqueen.Motors.ALL)
            basic.show_icon(IconNames.SQUARE)
            return
        # Arrête l'exécution de la fonction
        # Détection des obstacles avec le capteur à ultrasons
        distance = maqueen.ultrasonic()
        if distance < MIN_DISTANCE:
            maqueen.motor_stop(maqueen.Motors.ALL)
            return
        # Arrête l'exécution de la fonction pour éviter les collisions
        # Détection des capteurs de ligne
        if maqueen.read_patrol(maqueen.Patrol.PATROL_LEFT) == 1 and maqueen.read_patrol(maqueen.Patrol.PATROL_RIGHT) == 0:
            error = -1
        elif maqueen.read_patrol(maqueen.Patrol.PATROL_LEFT) == 0 and maqueen.read_patrol(maqueen.Patrol.PATROL_RIGHT) == 1:
            error = 1
        elif maqueen.read_patrol(maqueen.Patrol.PATROL_LEFT) == 1 and maqueen.read_patrol(maqueen.Patrol.PATROL_RIGHT) == 1:
            error = 0
        else:
            error = previous_error
        # Maintient de l'erreur précédente si aucune ligne détectée
        # PID
        integral += error
        integral = constrain(integral, 0 - INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        # Anti-windup
        derivative = error - previous_error
        correction = constrain(Kp * error + Ki * integral + Kd * derivative,
            0 - MAX_SPEED,
            MAX_SPEED)
        # Calcul des vitesses
        left_speed = constrain(base_speed - correction, MIN_SPEED, MAX_SPEED)
        right_speed = constrain(base_speed + correction, MIN_SPEED, MAX_SPEED)
        # Envoi aux moteurs
        if left_speed >= 0:
            maqueen.motor_run(maqueen.Motors.M1, maqueen.Dir.CW, left_speed)
        else:
            maqueen.motor_run(maqueen.Motors.M1, maqueen.Dir.CCW, abs(left_speed))
        if right_speed >= 0:
            maqueen.motor_run(maqueen.Motors.M2, maqueen.Dir.CW, right_speed)
        else:
            maqueen.motor_run(maqueen.Motors.M2, maqueen.Dir.CCW, abs(right_speed))
        # Enregistrement des données
        datalogger.log(datalogger.create_cv("left", left_speed))
        datalogger.log(datalogger.create_cv("right", right_speed))
        # Mise à jour de l'erreur précédente
        previous_error = error
basic.forever(on_forever)
