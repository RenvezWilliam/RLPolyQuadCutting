from random import randint


def create_shape(num):
    rectangle_to_create = randint(3, 15)
    r = rectangle_to_create

    recs = []

    #
    # Création du premier rectangle
    #

    x = 0
    y = 0
    dx = randint(3, 7)
    dy = randint(3, 7)

    recs.append([x, y, dx, dy])

    #
    # Création des rectangles suivants
    #

    for i in range(r - 1):
        rng = randint(0, len(recs) - 1)
        
        if recs[rng][0] + 1 <= recs[rng][0] + recs[rng][2] - 1:
            x = randint(recs[rng][0] + 1, recs[rng][0] + recs[rng][2] - 1)
        else:
            x = recs[rng][0] + 1
        
        if recs[rng][1] + 1 <= recs[rng][1] + recs[rng][3] - 1:
            y = randint(recs[rng][1] + 1, recs[rng][1] + recs[rng][3] - 1)
        else:
            y = recs[rng][1] + 1

        # 1 : dépassera sur l'axe X | 2 : dépassera sur l'axe Y | 3 dépassera sur les deux
        # (concerne uniquement le rectangle rng et celui qui est en train d'être construit)
        depassement = randint(1, 3)

        # Savoir si la valeur va dépasser vers le haut, bas, gauche, droite
        neg_x = randint(0, 1)
        neg_y = randint(0, 1)

        if depassement == 1 or depassement == 3:
            if neg_x == 0:
                dx = randint(recs[rng][2]-(x - recs[rng][0]), recs[rng][2]-(x-recs[rng][0])+5)
            else:
                dx = randint((x - recs[rng][0]), (x - recs[rng][0]) + 5)
        else:
            if neg_x == 0:
                if recs[rng][2]-(x - recs[rng][0]) >= 1:
                    dx = randint(1, recs[rng][2]-(x - recs[rng][0]))
                else:
                    dx = 1
            else:

                dx = randint(1, (x - recs[rng][0]))

        if depassement == 2 or depassement == 3:
            if neg_y == 0:
                dy = randint(recs[rng][3] - (y - recs[rng][1]), recs[rng][3] - (y - recs[rng][1]) + 5)
            else:
                dy = randint((y - recs[rng][1]), (y - recs[rng][1]) + 5)
        else:
            if neg_y == 0:
                if recs[rng][3] - (y - recs[rng][1]) >= 1:
                    dy = randint(1, recs[rng][3] - (y - recs[rng][1]))
                else:
                    dy = 1
            else:
                dy = randint(1, (y - recs[rng][1]))

        if dx == 0:
            dx = 1
        if dy == 0:
            dy = 1

        if neg_x == 1:
            x -= dx
        if neg_y == 1:
            y -= dy

        recs.append([x, y, dx, dy])

    # -=-=-=-=-=-
    # Création du fichier
    # -=-=-=-=-=-

    """ !!! Modifira les fichiers déjà créers portant le nom shape_X.txt (X étant un nombre) !!! """

    nom_fichier = "shape_"+str(num)+".txt"
    f = open(nom_fichier, "w")


    for i in range(len(recs)):
        if not i == len(recs) - 1:
            f.write(str(recs[i][0])+", "+str(recs[i][1])+", "+str(recs[i][2])+", "+str(recs[i][3])+"\n")
        else:
            f.write(str(recs[i][0]) + ", " + str(recs[i][1]) + ", " + str(recs[i][2]) + ", " + str(recs[i][3]))


    f.close()
    return True


if __name__ == '__main__':

    nbr_of_files = 30
    finished = False

    while not finished:

        if nbr_of_files <= 1:
            finished = True

        if create_shape(nbr_of_files):
            nbr_of_files = nbr_of_files - 1
