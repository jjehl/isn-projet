# coding:utf8 

import sys 
import numpy as np
import time
import os

x = sys.argv[1]
y = sys.argv[2]
z = sys.argv[3]
nb_objet = sys.argv[4]

from poppy.creatures import PoppyTorso
from cinematique import Bras,Buste
from couple import *

#instanciation du robot avec le nom marlien (Mar pour Martin et Lien pour Julien)
marlien = PoppyTorso() #Commande uniquement pour le vrai robot
marlien.compliant = False
#mise à 0 des moteurs
for m in marlien.motors:
    m.goto_position(0,2)
    

time.sleep(2)
coord_objet = np.mat([[float(x)],[float(y)],[float(z)]])
nb_objet = int(nb_objet)

marlien.l_shoulder_y.goto_position(-180,3)
marlien.l_elbow_y.goto_position(90,3)
marlien.r_shoulder_y.goto_position(-180,3)
marlien.r_elbow_y.goto_position(90,3,wait=True)

buste = Buste(coord_objet)

# la méthode alignement permet de calculer l'angle pour aligner le buste
rot_buste = buste.alignement()

# rotation du buste pour être face à l'objet
marlien.abs_z.goto_position(rot_buste,3,wait=True)

# instanciation des 2 bras du robots
bras_droit=Bras('droit',buste)
bras_gauche=Bras('gauche',buste)

# utilisation de la méthode analytique pour le bras gauche
angleg = bras_gauche.analytique()

# on positionne le bras gauche en utilisant la méthode analytique
marlien.l_shoulder_y.goto_position(angleg['shoulder_y'],2)
marlien.l_elbow_y.goto_position(angleg['elbow_y'],2)

angled = bras_droit.parcours_espace((-180,90),(-90,50),20.0)

# on positionne le bras droit en utilisant la méthode parcours espace
marlien.r_shoulder_y.goto_position(angled['shoulder_y'],2)
marlien.r_elbow_y.goto_position(angled['elbow_y'],2,wait=True)

#boucle sur le nombre d'objets à tester
dico_objet = {}

for i in range(1,nb_objet+1):
    os.system('espeak -p 70 -a 200 -s 100 -v fr-fr  "Merci de positionner un objet dans les 5 secondes"')
    time.sleep(5)
    # le robot va serrer l'objet
    t0 = time.time()
    load = []
    t = []
    marlien.l_arm_z.goto_position(-40,3)
    marlien.r_arm_z.goto_position(40,3)
    while time.time()-t0<3:
        load.append(marlien.l_arm_z.present_load-marlien.r_arm_z.present_load)
        t.append(time.time()-t0)
        time.sleep(0.02)

    marlien.l_arm_z.goto_position(0,1)
    marlien.r_arm_z.goto_position(0,1,wait=True)

    objet_analyse = Courbe(t,load)
    '''
    #debug
    import pickle
    with open('test_objet', 'wb') as fichier:
        mon_pickler = pickle.Pickler(fichier)
        mon_pickler.dump(t)
        mon_pickler.dump(load)
    '''
    dico_objet[objet_analyse.coef()]=i
    
liste_keys = list(dico_objet.keys())
Tri(liste_keys)

os.system('espeak -p 70 -a 200 -s 100 -v fr-fr  "Du plus mou au plus dur, nous avons : "')
for j in liste_keys:
    phrase = 'espeak -p 70 -a 200 -s 100 -v fr-fr  "numéro '+str(dico_objet[j])+' "'
    os.system(phrase)
    print (dico_objet[j])
    
    
marlien.compliant = True
[p.stop() for p in marlien.active_primitives]
marlien.close()
