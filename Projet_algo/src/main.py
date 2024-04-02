from const import *
import csv
import tkinter as tk
from tkinter import messagebox
from math import *
from PIL import ImageTk, Image
import time

class Main() :

  
    def __init__(self) :
        self.racine = tk.Tk()
        self.racine.title("Grille")
        self.racine.geometry("700x600")
        self.creer_widgets(self.racine)
        
        #variables de sélection initialisées à False
        self.mur_selec = False
        self.arrivee_selec = False
        self.depart_selec = False
        self.gomme_selec = False
        self.effacer = False
        
        #coordonnées du départ et de l'arrivée
        self.depart_coord = [-1, -1]
        self.arrivee_coord = [-1,-1]
        
        #initialisation des variables relatives au labyrinthe et au calcul
        self.adjacence = {}
        self.matrice_labyrinthe = []
        self.creer_matrice_labyrinthe_vierge()     
        self.chemin = []
        
        self.timer = 0

    def creer_widgets(self, root) :
        """
        fonction dans laquelle sont créés tous les widgets tkinter
        """
        self.fenetre = None
      
        self.bouton_calculer = tk.Button(root, text = "Calculer le chemin", bg = '#d4efdf')
        self.bouton_calculer.bind("<Button-1>", self.calcul)
        self.bouton_calculer.pack(side = "top", fill = "x")
        
        
        # on crée le canvas du labyrinthe
        self.canvas = tk.Canvas(self.racine, width=DIMENSION, height=DIMENSION)
        self.canvas.pack(side = tk.LEFT)
        self.grille()
        
        #widgets
        self.bouton1 = tk.Button(root, text = "Placer les murs", bg = 'lightgreen')
        self.bouton1.bind("<Button-1>", self.placement_murs)
        self.bouton1.pack(fill = 'x')
        
        self.bouton2 = tk.Button(root, text = "Placer le départ", bg = '#f7dc6f')
        self.bouton2.bind("<Button-1>", self.placement_depart)
        self.bouton2.pack(fill = 'x')
        
        self.bouton3 = tk.Button(root, text = "Placer l'arrivée", bg = '#f8c471')
        self.bouton3.bind("<Button-1>", self.placement_arrivee)
        self.bouton3.pack(fill = 'x')
        

        self.choix_etapes = tk.IntVar()
        self.check = tk.Checkbutton(root, text = "Afficher les étapes", onvalue=True, offvalue=False, variable = self.choix_etapes)
        self.check.pack()
        
        self.choix_algo = tk.IntVar()
        self.astar_selec_button = tk.Radiobutton(root, text="Astar", variable=self.choix_algo, value=0)
        self.astar_selec_button.pack()
        
        self.bfs_selec_button = tk.Radiobutton(root, text="BFS", variable=self.choix_algo, value=1)
        self.bfs_selec_button.pack()
  
        self.bouton_gomme = tk.Button(root, text = "Gomme", bg = '#abb2b9')
        self.bouton_gomme.bind("<Button-1>", self.gommer)
        self.bouton_gomme.pack(fill = 'x')
        
        self.bouton_reset = tk.Button(root, text = "Tout effacer", bg = '#e6b0aa')
        self.bouton_reset.bind("<Button-1>", self.reset)
        self.bouton_reset.pack(fill = "x")
        
        self.label = tk.Label(root, text = 'Temps de calcul :')
        self.label.pack()
        self.affichage_timer = tk.Label(root)
        self.affichage_timer.pack()
            
        self.bouton_chargement = tk.Button(root, text = "Charger un labyrinthe")
        self.bouton_chargement.bind("<Button-1>", self.ouvrir_fenetre)
        self.bouton_chargement.pack(side = 'bottom')
        
    def ouvrir_fenetre(self, event) :
        """ 
        crée la fenêtre TopLevel sur laquelle on choisira des labyrinthes pré-faits
        """
        if (self.fenetre != None):
            self.fenetre.destroy()
        self.fenetre = tk.Toplevel(self.racine)
        self.fenetre.title("Labyrinthes")
        self.fenetre.geometry(f"{F_HAUTEUR}x{F_LARGEUR}")
        self.fenetre.resizable(height = False, width = False)
        self.label1 = tk.Label(self.fenetre, text = "Choisissez un modèle de labyrinthe", font='Helvetica 10 bold')
        self.label1.pack()


        img = ImageTk.PhotoImage(Image.open("labyrinthes/labyrinthe1.jpg"))  # PIL solution
        self.labyrinthe1 = tk.Label(self.fenetre, image = img)
        self.labyrinthe1.image = img
        self.labyrinthe1.bind("<Button-1>", self.charg_labyrinthe1)
        self.labyrinthe1.pack()

        img = ImageTk.PhotoImage(Image.open("labyrinthes/labyrinthe2.jpg"))  # PIL solution
        self.labyrinthe2 = tk.Label(self.fenetre, image = img)
        self.labyrinthe2.image = img
        self.labyrinthe2.bind("<Button-1>", self.charg_labyrinthe2)
        self.labyrinthe2.pack()

        img = ImageTk.PhotoImage(Image.open("labyrinthes/labyrinthe3.jpg"))  # PIL solution
        self.labyrinthe3 = tk.Label(self.fenetre, image = img)
        self.labyrinthe3.image = img
        self.labyrinthe3.bind("<Button-1>", self.charg_labyrinthe3)
        self.labyrinthe3.pack()

        img = ImageTk.PhotoImage(Image.open("labyrinthes/labyrinthe4.jpg"))  # PIL solution
        self.labyrinthe4 = tk.Label(self.fenetre, image = img)
        self.labyrinthe4.image = img
        self.labyrinthe4.bind("<Button-1>", self.charg_labyrinthe4)
        self.labyrinthe4.pack()

    def charg_labyrinthe1(self,event) :
        """
        fonction callback appelée quand on clique sur l'image du labyrinthe 1
        lie l'image au bon fichier csv et appelle la fonction chargement pour charger le fichier
        """
        self.path = "labyrinthes/matrice1.csv"
        self.chargement()
    
    def charg_labyrinthe2(self,event) :
        """
        fonction callback appelée quand on clique sur l'image du labyrinthe 2
        pareil que char_labyrinthe1()
        """
        self.path = "labyrinthes/matrice2.csv"
        self.chargement()

    def charg_labyrinthe3(self,event) :
        """
        fonction callback appelée quand on clique sur l'image du labyrinthe 3
        pareil que char_labyrinthe1()
        """
        self.path = "labyrinthes/matrice3.csv"
        self.chargement()
    
    def charg_labyrinthe4(self,event) :
        """
        fonction callback appelée quand on clique sur l'image du labyrinthe 4
        pareil que char_labyrinthe1()
        """
        self.path = "labyrinthes/matrice4.csv"
        self.chargement()
        
    def placement_murs(self, event) :
        """
        fonction callback appelée quand on appuie sur le bouton 'Placer les murs'
        passe toutes les variables de sélection à False sauf self.mur_selc qui passe à True
        """
        self.mur_selec = True
        self.arrivee_selec = False
        self.depart_selec = False
        self.gomme_selec = False
    
    def placement_depart(self,event):
        """
        fonction callback appelée quand on appuie sur le bouton 'Placer le départ'
        passe toutes les variables de sélection à False sauf self.depart_selec qui passe à True
        """
        self.mur_selec = False
        self.arrivee_selec = False
        self.depart_selec = True
        self.gomme_selec = False
    
    def placement_arrivee(self, event):
        """
        fonction callback appelée quand on appuie sur le bouton 'Placer l'arrivée'
        passe toutes les variables de sélection à False sauf self.arrivee_selec qui passe à True
        """
        self.mur_selec = False
        self.arrivee_selec = True
        self.depart_selec = False
        self.gomme_selec = False

    def gommer(self, event):
        """
        fonction callback appelée quand on appuie sur le bouton 'Gommer'
        passe toutes les variables de sélection à False sauf self.gomme_selec qui passe à True
        """
        self.mur_selec = False
        self.arrivee_selec = False
        self.depart_selec = False
        self.gomme_selec = True
    
    def chargement(self):
        """
        charge le fichier csv et met à jour la matrice self.matrice_labyrinthe 
        puis appelle la fonction grilleFromcsv() pour l'afficher dans le canvas
        """
        self.mat = []
        with open(self.path, newline='') as csvfile:
            lecteur = csv.reader(csvfile, delimiter=',')
            for ligne in lecteur :
                l = []
                for i in range(len(ligne)) :
                    l.append(int(ligne[i]))
                self.mat.append(l)
        self.matrice_labyrinthe = self.mat
        self.grilleFromcsv()
        self.fenetre.destroy()

    def grilleFromcsv(self) :
        """
        affiche la grille à partir de la matrice self.matrice_labyrinthe
        """
        self.depart_coord = [-1, -1] #remet les valeurs de base de départ et d'arrivée
        self.arrivee_coord = [-1,-1]
        for x in range(len(self.matrice_labyrinthe)) :
            for y in range(len(self.matrice_labyrinthe)) :
                item = self.canvas.find_closest(y*TAILLE_CASE, x*TAILLE_CASE)
                if self.matrice_labyrinthe[x][y] == 0 :      #si case = 0 : case vide => bleu clair
                    self.canvas.itemconfigure(item, fill = "lightblue")
                elif self.matrice_labyrinthe[x][y] == 1  :  #si case = 1 : mur => vert
                    self.canvas.itemconfigure(item, fill = "green")
                elif self.matrice_labyrinthe[x][y] == -1  : #si case = -1 : départ => jaune
                    self.canvas.itemconfigure(item, fill = "#f1c40f")
                    self.depart_coord[0] = x #met à jour les coordonnées du départ
                    self.depart_coord[1] = y
                elif self.matrice_labyrinthe[x][y] == -2 : #si case = -2 : arrivée => rouge
                    self.canvas.itemconfigure(item, fill = "#d35400")
                    self.matrice_labyrinthe[x][y] = -1
                    self.arrivee_coord[0] = x  #met à jour les coordonnées de l'arrivée
                    self.arrivee_coord[1] = y

                  

    def calcul(self, event):
        """
        fonction callback appelée quand on appuie sur le bouton calculer
        appelle self.effacer_apres_calcul_chemin() pour effacer le précédent chemin
        puis appelle self.creer_dict_adjacence() pour créer le dictionnaire d'ajdacence
        puis appelle self.Astar() ou self.BFS() qui calcule le chemin
        puis appelle self.affiche_chemin() pour afficher le chemin sur le canvas
        """
        self.effacer_apres_calcul_chemin()
        if self.depart_coord[0] == -1 or self.arrivee_coord[0] == -1 :
            tk.messagebox.showinfo("Erreur", "Vous essayez de calculer un chemin sans avoir placé le depart ou l'arrivée. Utilisez les boutons à droite.")
        else :
            self.creer_dict_adjacence()
            if self.choix_algo.get() == 0 : #si la case Astar est cochée
                self.Astar()
            elif self.choix_algo.get() == 1 : #si la case BFS est cochée
                self.BFS()
            self.affiche_chemin()
      
    def reset(self, event):
        """
        fonction callback appelée quand on appuie sur le bouton 'Tout effacer'
        rend toutes les cases vides en recréant une matrice self.matrice_labyrinthe remplie de 0 et en l'affichant avec self.grille()
        """
        for x in range(LIGNE):
            for y in range(COLONNE):
                self.matrice_labyrinthe[x][y] = 0
        self.depart_coord = [-1, -1] #on remet les coordonnées du départ et d'arrivée à leur valeur initiale car il n'y en a plus sur la grille
        self.arrivee_coord = [-1, -1]
        self.grille()
                                                 
        
    def grille(self) :
        """
        crée la grille sur le canvas
        """
        y = 0
        while y < DIMENSION : 
            x = 0
            while x < DIMENSION :
                self.case_id = self.canvas.create_rectangle(x,y,x+TAILLE_CASE,y+TAILLE_CASE,fill='lightblue')  #crée une case
                self.canvas.bind("<B1-Motion>", self.creer_el) #permet de pouvoir interragir avec les cases en restant appuyé
                self.canvas.bind("<Button-1>", self.creer_el) #ou juste en cliquant
                x+=TAILLE_CASE
            y+=TAILLE_CASE
        
    def effacer_apres_calcul_chemin(self):
        """
        efface s'il le faut le chemin calculé par l'algorithme après un clic sur le canvas
        """
        if self.effacer :  #s'il faut effacer, i.e : si un chemin est affiché 
            for x in range(LIGNE) :
                for y in range(COLONNE) :
                    item_a_effacer = self.canvas.find_closest(y*TAILLE_CASE, x*TAILLE_CASE)
                    if self.matrice_labyrinthe[x][y] == 0 :
                        self.canvas.itemconfigure(item_a_effacer, fill = "lightblue") #remet la case en bleu clair
            self.effacer = False #remet la variable à False car on a effacé

    def creer_el(self, event):
        """
        fonction callback appelée quand on clique ou reste appuyé sur une case de la grille
        met à jour la matrice quand on place un élément et l'affiche sur le canvas
        """

        self.effacer_apres_calcul_chemin() #efface le chemin si nécessaire

        item_clicked = self.canvas.find_closest(event.x, event.y) #on récupère les coordonnées de la case cliquée
        y_norme = floor(event.x//TAILLE_CASE) #normalise les coordonnées de la case pour avoir sa ligne et sa colonne
        x_norme = floor(event.y//TAILLE_CASE)

        #vérification que la case est bien dans le canvas et si non on prend la valeur qu'on avait avant la sortie
        if event.x >= DIMENSION :
            y_norme = DIMENSION // TAILLE_CASE - 1
        if event.x <= 0 :
            y_norme = 0
        if event.y >= DIMENSION :
            x_norme = DIMENSION // TAILLE_CASE - 1
        if event.y <= 0 :
            x_norme = 0

        val_matrice = self.matrice_labyrinthe[x_norme][y_norme]
        if val_matrice == 0 :
            if self.mur_selec : # cas où on place les murs 
                self.matrice_labyrinthe[x_norme][y_norme] = 1 #les murs auront une valeur de 1 dans la matrice
                self.canvas.itemconfigure(item_clicked, fill = "green") #on les met vert
            
            elif self.arrivee_selec : # cas où on place une arrivee
                #on enlève l'ancienne arrivée si elle existe
                if self.arrivee_coord[0] != -1 :
                    old_x = self.arrivee_coord[0]
                    old_y = self.arrivee_coord[1]
                    self.matrice_labyrinthe[old_x][old_y] = 0
                    item_a_effacer = self.canvas.find_closest(old_y*TAILLE_CASE, old_x*TAILLE_CASE)
                    self.canvas.itemconfigure(item_a_effacer, fill = "lightblue")
                self.arrivee_coord[0] = x_norme #met à jour les coordonnées de l'arrivée
                self.arrivee_coord[1] = y_norme
                self.matrice_labyrinthe[x_norme][y_norme] = -1 #l'arrivée aura une valeur de -1 dans la matrice
                self.canvas.itemconfigure(item_clicked, fill = "#d35400") #on la met en rouge
            
            elif self.depart_selec : # cas où on place le depart
                #on enlève l'ancien départ si il existe
                if self.depart_coord[0] != -1 :
                    old_x = self.depart_coord[0]
                    old_y = self.depart_coord[1]
                    self.matrice_labyrinthe[old_x][old_y] = 0
                    item_a_effacer = self.canvas.find_closest(old_y*TAILLE_CASE, old_x*TAILLE_CASE)
                    self.canvas.itemconfigure(item_a_effacer, fill = "lightblue")
                self.depart_coord[0] = x_norme #met à jour les coordonnées du départ
                self.depart_coord[1] = y_norme
                self.matrice_labyrinthe[x_norme][y_norme] = -1  #le départ aura une valeur de -1 dans la matruce
                self.canvas.itemconfigure(item_clicked, fill = "#f1c40f") #on le met en jaune
        
        if self.gomme_selec and val_matrice != 0 : # cas où on a la gomme
            self.matrice_labyrinthe[x_norme][y_norme] = 0 #la valeur revient à 0
            self.canvas.itemconfigure(item_clicked, fill = "lightblue") #on remet la case en bleu clair
            if (x_norme == self.depart_coord[0] and y_norme == self.depart_coord[1]): #si on efface le départ
                self.depart_coord[0] = -1 #on remet les coordonnées du départ à leur valeur initiale car il n'y en a plus sur la grille
                self.depart_coord[1] = -1
            elif x_norme == self.arrivee_coord[0] and y_norme == self.arrivee_coord[1]: #si on efface l'arrivée
                self.arrivee_coord[0] = -1 #on remet les coordonnées d'arrivée à leur valeur initiale car il n'y en a plus sur la grille
                self.arrivee_coord[1] = -1
            
    def creer_matrice_labyrinthe_vierge(self):
        """
        crée la matrice 2D correspondante au labyrinthe de taille n x n et remplie de 0
        """
        for i in range(LIGNE):
            line = []
            for j in range(COLONNE):
                line.append(0) #les sommets sont tous à zero de base
            self.matrice_labyrinthe.append(line)
            
    def creer_dict_adjacence(self):
        """
        met à jour le dictionnaire self.adjacence dans lequel on associe à une case (clé) à une liste des cases vides voisines (valeurs)
        """
        for x in range(LIGNE):
            for y in range(COLONNE):
                sommet_courant = (x, y)
                self.adjacence[sommet_courant] = list()
                
                if self.matrice_labyrinthe[x][y]!= 1 :
                    # On assigne au sommet codé, ses sommets adjacents
                    if x>0 and self.matrice_labyrinthe[x-1][y] != 1 :
                        self.adjacence[sommet_courant].append((x-1, y))
                    if x<LIGNE-1 and self.matrice_labyrinthe[x+1][y] != 1 :
                        self.adjacence[sommet_courant].append((x+1, y))
                    if y>0 and self.matrice_labyrinthe[x][y-1] != 1 :
                        self.adjacence[sommet_courant].append((x, y-1))
                    if y<COLONNE-1 and self.matrice_labyrinthe[x][y+1] != 1 :
                        self.adjacence[sommet_courant].append((x, y+1))

    
    # Partie de calcul de chemin
    def pop_min(self, file):
        """
        enlève de la file le voisin avec l'heuristique la plus faible
        """
        cle_min = list(file.keys())[0]
        for key, value in file.items():
            if float(value) <= float(file[cle_min]):
                cle_min = key
        cle_retour = file.pop(cle_min)
        return cle_min, cle_retour


    def heuristique(self, depart, arrivee):
        """
        heuristique basée sur la distance cartésienne entre le départ et l'arrivée pris en paramètres
        """
        x1, y1 = depart[0], depart[1]
        x2, y2 = arrivee[0], arrivee[1] 
        return sqrt((x1-x2)**2+(y1-y2)**2)

    

    def BFS(self):
        """
        algorithme de parcours de graphe en largeur qui calcule le chemin entre le départ et l'arrivée
        """
        if self.choix_etapes.get() == 1 :
            self.affichage_timer['text'] = ""
        
        #initialisation des variables et du timer
        last_time = time.time()
        depart = (self.depart_coord[0], self.depart_coord[1])
        arrivee = (self.arrivee_coord[0], self.arrivee_coord[1])
        file = [depart]
        deja_explore = [depart]
        
        predecesseur = {depart : depart}
        
        while len(file)!=0 : #tant que la file n'est pas vide
            courant = file.pop(0) #on prend le premier
            
            if courant == arrivee : #si on a trouvé l'arrivée on arrête
                break
            
            if courant != depart and courant != arrivee and self.choix_etapes.get() == 1 : #coloriage des cases explorées (si sélectionné par l'utilisateur)
                item_a_colorier = self.canvas.find_closest(courant[1]*TAILLE_CASE, courant[0]*TAILLE_CASE)
                self.canvas.itemconfigure(item_a_colorier, fill = "#ff2d49")
                self.canvas.update()
                time.sleep(0.05)
            
            for sommet in self.adjacence[courant]: #pour tous les sommets voisins
                if sommet not in deja_explore : #s'ils n'ont pas déjà été explorés
                    
                    if sommet != arrivee and self.choix_etapes.get() == 1 : #coloriage des sommets qu'on va ajouter à la file (si sélectionné par l'utilisateur)
                        item_a_colorier = self.canvas.find_closest(sommet[1]*TAILLE_CASE, sommet[0]*TAILLE_CASE) 
                        self.canvas.itemconfigure(item_a_colorier, fill = "#16a085")
                        self.canvas.update()
                        time.sleep(0.05)
                    
                    
                    file.append(sommet) #on ajoute le sommet à la file
                    deja_explore.append(sommet) #on le marque comme étant déjà exploré
                    predecesseur[sommet] = courant #et le prédecesseur de ce sommet et la case courante
        
        self.chemin = [arrivee]
        possible = True

        #on vérifie qu'on a bien trouvé l'arrivée
        while self.chemin[0]!=depart and possible :
            if predecesseur.get(self.chemin[0], -10) == -10:
                possible = False
            else :
                self.chemin.insert(0, predecesseur[self.chemin[0]]) #si oui on ajoute tout le chemin
    
        if not(possible) : #si non on affiche une erreur
            tk.messagebox.showinfo("Erreur", "Il n'existe pas de chemin entre le départ et l'arrivé.")
        
        if self.choix_etapes.get() == 0 : #gestion du timer
            self.timer = (time.time() - last_time)*1000 #temps en ms
            self.affichage_timer['text'] = f"{self.timer:3f} ms"


    def Astar(self):
        """
        algorithme de parcours de graphe qui utilise l'heuristique pour se guider et qui calcule le chemin entre le départ et l'arrivée
        """
        if self.choix_etapes.get() == 1 :
            self.affichage_timer['text'] = ""
        
        #initialisation des variables et du timer
        last_time = time.time()
        depart = (self.depart_coord[0], self.depart_coord[1])
        arrivee = (self.arrivee_coord[0], self.arrivee_coord[1]) 
        file = {depart: self.heuristique(depart, arrivee)}
        predecesseur = {}
        distance_arcs = {depart:0}
        deja_explore = {}
        
        courant = depart
        

        while len(file) != 0 : #tant que la file n'est pas vide
            courant, f_courant = self.pop_min(file) #on prend celui avec l'heuristique la plus faible

            if courant == arrivee : #si on a trouvé l'arrivée on arrête
                break
            
            if courant != depart and self.choix_etapes.get() == 1 : #coloriage des cases explorées (si sélectionné par l'utilisateur)
                item_a_colorier = self.canvas.find_closest(courant[1]*TAILLE_CASE, courant[0]*TAILLE_CASE)
                self.canvas.itemconfigure(item_a_colorier, fill = "#ff2d49")
                self.canvas.update()
                time.sleep(0.05)
            
            distance = distance_arcs[courant]
            deja_explore[courant]= distance
            for s in self.adjacence[courant]: #pour tous les sommets voisins
                via_courant = distance + 1
                if s not in deja_explore.keys() and via_courant < file.get(s, inf): #s'ils n'ont pas déjà été explorés et si la distance passant par le sommet est plus faible que celle qui lui est attribuée dans la file
                    
                    if s != arrivee and self.choix_etapes.get() == 1 : #coloriage des sommets qu'on va ajouter à la file (si sélectionné par l'utilisateur)
                        item_a_colorier = self.canvas.find_closest(s[1]*TAILLE_CASE, s[0]*TAILLE_CASE)
                        self.canvas.itemconfigure(item_a_colorier, fill = "#16a085")
                        self.canvas.update()
                        time.sleep(0.05)
                    
                    
                    file[s] = via_courant + self.heuristique(s, arrivee)
                    predecesseur[s] = courant
                    distance_arcs[s] = via_courant


        self.chemin = [arrivee]
        possible = True

        #on vérifie qu'on a bien trouvé l'arrivée
        while self.chemin[0]!=depart and possible :
            if predecesseur.get(self.chemin[0], -10) == -10:
                possible = False
            
            else :
                self.chemin.insert(0, predecesseur[self.chemin[0]])  #si oui on ajoute tout le chemin
        
        if not(possible) :
            tk.messagebox.showinfo("Erreur", "Il n'existe pas de chemin entre le départ et l'arrivé.") #si non on affiche une erreur
        
        if self.choix_etapes.get() == 0 : #gestion du timer
            self.timer = (time.time() - last_time)*1000 #temps en ms
            self.affichage_timer['text'] = f"{self.timer:3f} ms"
    
    def affiche_chemin(self):
        """
        fonction qui colorie les cases du chemin trouvé par l'algortihme en rose
        """
        for coord in self.chemin : #on prend les coordonnées des cases dans le chemin
            x = coord[0]
            y = coord[1]
            if ((x,y)!= (self.depart_coord[0],self.depart_coord[1])) and ((x,y)!= (self.arrivee_coord[0],self.arrivee_coord[1])): #si la case n'est pas le départ ou l'arrivée
                item_a_colorier = self.canvas.find_closest(y*TAILLE_CASE, x*TAILLE_CASE)
                self.canvas.itemconfigure(item_a_colorier, fill = "#ff5a98") #on colorie la case en rose
        self.effacer = True #permet dans cree_el() d'effacer le chemin calculé après un clic sur canvas
        
if __name__ == "__main__" : #lance le programme
    app = Main()
    app.racine.mainloop()