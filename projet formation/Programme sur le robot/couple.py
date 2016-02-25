# -*-coding:utf-8 -*

class Courbe :
    def lissage(self,nb_points):
        self.temps_lisse=[]
        self.couple_lisse=[]
        l=[]
        for i in self.couple:
            if len(l)<nb_points :
                l.append(i)
            else :
                s=0
                for j in l :
                    s += j
                s = s/nb_points
                self.couple_lisse.append(s)
                l.append(i)
                l.pop(0)

        self.temps_lisse = list(self.temps)
        for i in range(nb_points):
            self.temps_lisse.pop(0)
    
    def __init__(self,temps,couple):
        self.temps = temps
        self.couple = couple
        self.lissage(10)
    
    def etude_croissance(self):
        self.d_couple_lisse=[]
        self.d_temps_lisse=[]
        s = 0
        flag = False
        zone = [0,0]
        n=0
        for i1,t in zip(self.couple_lisse,self.temps_lisse):
            if s == 0 :
                i0 = i1
                s=1
            else :
                d = i1-i0
                self.d_couple_lisse.append(d)
                i0=i1
                self.d_temps_lisse.append(t)
                n+=1
                # recherche de la plus longue periode de croissance
                if flag==False :
                    if d>0 :
                        t0,n0 = t,n
                        flag = True
                else :
                    if d>0 : t1,n1=t,n
                    else : 
                        flag = False
                        if t1-t0 > zone[1]-zone[0] :zone = [t0,t1,n0,n1] 
        return zone
    
    def coef(self):
        s = 0
        zone = self.etude_croissance()
        for i in range(zone[2],zone[2]+25):
            s += self.couple_lisse[i]
        return int(s)
        
        
def Tri(tableau):

    for k in range (0,len(tableau)):

        min=tableau[k]
        imin=k #numero de la case ou se trouve le minimum
    
        for i in range (k+1,len(tableau)):

            if tableau[i]<min:

                min=tableau[i]
                imin=i

# On inverse les valeurs en utilisant une zone tampon temporaire

        tampon=tableau[imin]
        tableau[imin]=tableau[k]
        tableau[k]=tampon

    return tableau