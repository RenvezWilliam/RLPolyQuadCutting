import random
from math import *
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from pickle import *
import os.path
import datetime

import gmsh

""""global value to indicate how to round floating numbers"""
nb_digit_rounding = 4


def initialize():
    """ Initialize the gmsh module"""
    gmsh.initialize()
    gmsh.clear()
    # to avoid messages on the console
    gmsh.option.setNumber("General.Verbosity", 0)
    # gmsh.option.setNumber("Mesh.Algorithm", 3) # minimal delaunay
    # 11 <- quad frame based


def create_rectangle(x, y, dx, dy):
    """ create a 2D rectangle from left bottom corner (x,y) and
    side size dx in X direction and dy in Y direction.
    dx and dy must be strictly positive"""
    assert dx > 0
    assert dy > 0
    return gmsh.model.occ.add_rectangle(x, y, 0, dx, dy)


def fuse(pieces):
    """ fuses the input given list of pieces to return the
    Boolean union of all of those pieces """
    tool = [(2, pieces[0])]
    objs = []
    for i in range(1, len(pieces)):
        objs.append((2, pieces[i]))
    op = gmsh.model.occ.fuse(objs, tool)
    gmsh.model.occ.synchronize()
    return op[0][0][1]


def get_cell_tags(dim):
    info = gmsh.model.occ.getEntities(dim)
    tags = []
    for i, j in info:
        tags.append(j)
    return tags


def get_point_tags():
    """" returns the tags of all the vertices"""
    return get_cell_tags(0)


def get_edge_tags():
    """" returns the tags of all the edges"""
    return get_cell_tags(1)


def get_face_tags():
    """" returns the tags of all the faces"""
    return get_cell_tags(2)


def get_end_points(curve_tag):
    """ returns the two end points (extremities) of an edge"""
    data = gmsh.model.getAdjacencies(1, curve_tag)
    return data[1]


class Query:
    def __init__(self):
        self.__points = get_point_tags()
        self.__edges = get_edge_tags()
        self.__l2p = []

    def adjacent_curves(self, point_tag):
        curves = []
        for adj in self.__l2p:
            if adj[1] is point_tag:
                curves.append(adj[0])
        return curves

    def adjacent_points(self, curve_tag):
        points = []
        for adj in self.__l2p:
            if adj[0] is curve_tag:
                points.append(adj[1])
        return points

    def get_curves(self, face_tag):
        return gmsh.model.occ.get_curve_loops(face_tag)[1][0]

    def get_corners(self, face_tag):
        curves = gmsh.model.getAdjacencies(2, face_tag)[1]
        all_points = get_point_tags()
        corners = []
        for i_curve in range(0, len(curves)):
            j_index = i_curve - 1
            if i_curve == 0:
                j_index = len(curves) - 1
            prev_pnts = get_end_points(curves[j_index])
            current_pnts = get_end_points(curves[i_curve])
            common_pnt = 0
            if prev_pnts[0] == current_pnts[0]:
                common_pnt = current_pnts[0]
            elif prev_pnts[0] == current_pnts[1]:
                common_pnt = current_pnts[1]
            elif prev_pnts[1] == current_pnts[0]:
                common_pnt = current_pnts[0]
            else:
                common_pnt = current_pnts[1]
            corners.append(common_pnt)

        return corners


def point_coordinate(tag):
    """:return (x,y) the location of the point 'tag'"""
    bb = gmsh.model.occ.getBoundingBox(0, tag)
    return (round(bb[0], nb_digit_rounding),
            round(bb[1], nb_digit_rounding))


def cut(point_tag, face_tag, dir):
    """ Cut the face face_tag starting from point-tag in direction dir
        dir can be 1 (north), 2 (south), 3 (east), 4 (west)  """
    to_infinity = 0
    coord = point_coordinate(point_tag)
    if dir == 1:
        to_infinity = gmsh.model.occ.add_point(coord[0], coord[1] + 100, 0)
    elif dir == 2:
        to_infinity = gmsh.model.occ.add_point(coord[0], coord[1] - 100, 0)
    elif dir == 3:
        to_infinity = gmsh.model.occ.add_point(coord[0] + 100, coord[1], 0)
    elif dir == 4:
        to_infinity = gmsh.model.occ.add_point(coord[0] - 100, coord[1], 0)

    cut_edge = gmsh.model.occ.addLine(point_tag, to_infinity)
    gmsh.model.occ.fragment([(2, face_tag)], [(1, cut_edge)])
    gmsh.model.occ.synchronize()
    for e in get_edge_tags():
        if len(gmsh.model.getAdjacencies(1, e)[0]) == 0:
            gmsh.model.occ.remove([(1, e)], True)
    gmsh.model.occ.synchronize()
    return to_infinity


def finalize():
    """ Finalize the gmsh call """
    gmsh.fltk.run()
    gmsh.finalize()


def center(face_tag):
    return gmsh.model.occ.getCenterOfMass(2, face_tag)


def area(face_tag):
    return gmsh.model.occ.getMass(2, face_tag)


def remesh():
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.clear([])
    gmsh.model.mesh.generate(2)  # gives the coarse mesh only


def print_infos():
    print("---------------------------------")
    print("Faces ", get_face_tags())
    for f_tag in get_face_tags():
        q = Query()
        print("Face ", f_tag, ", center ", center(f_tag), ", and area ", area(f_tag))
        print(" --> curves ", q.get_curves(f_tag))
        print(" --> points ", q.get_corners(f_tag))


# === Mes ajouts === #

def is_a_rectangle(f_tag, print_info=False):
    q = Query()

    points = q.get_corners(f_tag)

    min_x = point_coordinate(points[0])[0]
    min_y = point_coordinate(points[0])[1]
    max_x = point_coordinate(points[0])[0]
    max_y = point_coordinate(points[0])[1]

    # Obtiens les valeurs minimums et maximums de x et y

    for p in points:
        if min_x > point_coordinate(p)[0]:
            min_x = point_coordinate(p)[0]
        if min_y > point_coordinate(p)[1]:
            min_y = point_coordinate(p)[1]
        if max_x < point_coordinate(p)[0]:
            max_x = point_coordinate(p)[0]
        if max_y < point_coordinate(p)[1]:
            max_y = point_coordinate(p)[1]

    # Maintenant qu'on à les valeurs minimums, on regarde si tous les points de la figure contiennent au moins une des
    # valeurs, si ce n'est pas le cas, ce n'est pas un rectangle.

    if print_info:
        print("-=-=-= Info =-=-=-")
        print("f_tag :", f_tag)
        print("min_x :", min_x)
        print("min_y :", min_y)
        print("max_x :", max_x)
        print("max_y :", max_y)
        print("------------")
        for p in points:
            print("point trouvé à", point_coordinate(p))

    for p in points:
        if min_x == point_coordinate(p)[0] or min_y == point_coordinate(p)[1] or max_x == point_coordinate(p)[0] or max_y == point_coordinate(p)[1]:
            continue
        return False

    return True


def is_on_a_face(point, direction, f_tag):
    q = Query()
    point_test = None
    coord = point_coordinate(point)
    if direction == 1:
        point_test = (coord[0], coord[1] + 0.5)
    elif direction == 2:
        point_test = (coord[0], coord[1] - 0.5)
    elif direction == 3:
        point_test = (coord[0] + 0.5, coord[1])
    elif direction == 4:
        point_test = (coord[0] - 0.5, coord[1])

    points = q.get_corners(f_tag)

    coords = []
    for p in points:
        coords.append(point_coordinate(p))

    point = Point(point_test)
    polygon = Polygon(coords)

    return polygon.covers(point), polygon.contains(point)


def create_shape(file):
    # créer un tableau avec les différents rectangle inscrit sur le fichier et transforme le tout en int
    lines = file.read().split('\n')
    for i in range(len(lines)):
        lines[i] = lines[i].split(',')
        for j in range(len(lines[i])):
            lines[i][j] = int(lines[i][j])
    # print(lines)
    # Créer tous les rectangles à partir du tableau obtenu juste avant
    rectangles = []
    for i in range(len(lines)):
        rectangles.append(create_rectangle(lines[i][0], lines[i][1], lines[i][2], lines[i][3]))
    del lines
    return fuse(rectangles)


def algo():
    # Récupère la surface initiale
    faces = get_face_tags()

    while faces:
        f = random.choice(faces)  # Prend une face aléatoirement
        points_candidats = get_cutable_pts(f)  # Récupère les points candidats pour être coupé
        p = random.choice(points_candidats)  # Récupère un point
        c = random.choice(can_cut(p, f))  # Récupère une découpe possible

        cut(p, f, c)  # Fait la découpe

        faces = get_all_non_square_faces()


def get_all_non_square_faces():
    values = []
    faces = get_face_tags()
    #print("-=-=-=-=-=-")
    for f in faces:
        #print(f, "is a rectangle :", is_a_rectangle(f))
        if not is_a_rectangle(f):
            values.append(f)
    return values

def get_taille(f_tag):
    q = Query()

    points = q.get_corners(f_tag)

    min_x = point_coordinate(points[0])[0]
    min_y = point_coordinate(points[0])[1]
    max_x = point_coordinate(points[0])[0]
    max_y = point_coordinate(points[0])[1]

    # Obtiens les valeurs minimums et maximums de x et y

    for p in points:
        if min_x > point_coordinate(p)[0]:
            min_x = point_coordinate(p)[0]
        if min_y > point_coordinate(p)[1]:
            min_y = point_coordinate(p)[1]
        if max_x < point_coordinate(p)[0]:
            max_x = point_coordinate(p)[0]
        if max_y < point_coordinate(p)[1]:
            max_y = point_coordinate(p)[1]

    #print(f_tag, " : ", max_x, min_x, " | ", max_y, min_y)
    return [max_x - min_x, max_y - min_y]


def get_cutable_pts(f_tag):
    q = Query()
    pts_list = q.get_corners(f_tag)
    values = []

    for p in pts_list:
        if can_cut(p, f_tag):
            values.append(p)

    return values


def launch():
    algo()
    print("score =", get_score())


def get_dist(point_tag1, point_tag2):
    # renvoi la distance entre 2 points

    coordPoint1 = point_coordinate(point_tag1)
    coordPoint2 = point_coordinate(point_tag2)

    return sqrt((coordPoint1[0] - coordPoint2[0]) ** 2 + (coordPoint1[1] - coordPoint2[1]) ** 2)


def get_dir(point_tag1, point_tag2):
    # renvoi la direction d'un point 2 par rapport a un point 1
    """dir = 1 = nord | 2 = sud | 3 = est | 4 = ouest"""

    no_parametrization = []
    [x1, y1, z1] = gmsh.model.getValue(0, point_tag1, no_parametrization)

    no_parametrization = []
    [x2, y2, z2] = gmsh.model.getValue(0, point_tag2, no_parametrization)

    if y2 > y1:
        return 1

    if y1 > y2:
        return 2

    if x2 > x1:
        return 3

    if x1 > x2:
        return 4

    return False

def get_score():
    q = Query()
    ratio = 0

    for f_tag in get_face_tags():

        taille = get_taille(f_tag)

        ratio = ratio + abs(taille[0] - taille[1])

    ratio = ratio / len(get_face_tags())

    return ratio


def can_cut(point, f_tag):
    dir = []
    for i in range(4):
        j = i + 1
        if is_on_a_face(point, j, f_tag)[1]:
            dir.append(j)
    return dir

def get_ref_point(point, ListFace, Face, c):

    """
    revoie a un tableaux contenant:

        - avec longueur = longueur du rectangle englobant toute les face de Face et largeur = largeur du rectangle englobant toute les face de ListFace
                si longueur  > largeur => 1;  si longueur  < largeur => 2;    si longueur  = largeur => 3

        - liste des directions ou il est possible de couper

        - direction du centre fonction center
    """
    q = Query()
    ref = []
    FinalRef = ""
    l = get_encompassing(ListFace)

    if l[0] == l[1]:
        ref.append(3)
    elif l[0] < l[1]:
        ref.append(2)
    else:
        ref.append(1)

    print(point, Face)
    ref.append(can_cut(point, Face))

    Point_x = point_coordinate(point)[0]
    Point_y = point_coordinate(point)[1]

    dir = []

    if c[1] > Point_y:
        dir.append(1)

    if Point_y > c[1]:
        dir.append(2)

    if c[0] > Point_x:
        dir.append(3)

    if Point_x > c[0]:
        dir.append(4)

    ref.append(dir)

    for r in ref:
        FinalRef = FinalRef + str(r) + "/"

    return FinalRef

def get_ref_point_deterministe(point, ListFace, c):

    """
    revoie a un tableaux contenant:

        - avec longueur = longueur du rectangle englobant toute les face de Face et largeur = largeur du rectangle englobant toute les face de ListFace
                si longueur  > largeur => 1;  si longueur  < largeur => 2;    si longueur  = largeur => 3

        - liste des directions ou il est possible de couper

        - direction du centre fonction center
    """
    q = Query()
    ref = []
    FinalRef = ""
    l = get_encompassing(ListFace)
    Face = None

    for f in ListFace:
        if Face != None:
            break
        for p in q.get_corners(f):
            if p == point:
                Face = f
                break
    ref.append(can_cut(point, Face))

    Point_x = point_coordinate(point)[0]
    Point_y = point_coordinate(point)[1]

    dir = []

    if c[1] > Point_y:
        dir.append(1)

    if Point_y > c[1]:
        dir.append(2)

    if c[0] > Point_x:
        dir.append(3)

    if Point_x > c[0]:
        dir.append(4)

    ref.append(dir)

    for r in ref:
        FinalRef = FinalRef + str(r) + "/"

    return FinalRef

def get_encompassing(ListFace):

    """
    Renvoie la longeur et la largeur du rectangle englobant toute les face de ListFace
    """

    if len(ListFace) == 0:
        return False

    q = Query()
    point = q.get_corners(ListFace[0])[0]
    Min_x = point_coordinate(point)[0]
    Min_y = point_coordinate(point)[1]
    Max_x = point_coordinate(point)[0]
    Max_y = point_coordinate(point)[1]

    for f in ListFace:
        for p in q.get_corners(f):
            coordP = point_coordinate(p)
            if coordP[0] > Max_x:
                Max_x = coordP[0]
            if coordP[1] > coordP[1]:
                Max_y = coordP[1]
            if coordP[0] < Min_x:
                Min_x = coordP[0]
            if coordP[1] < Min_y:
                Min_y = coordP[1]

    return [Max_x - Min_x, Max_y - Min_y]

def algo_random():
    # Récupère la surface initiale
    faces = get_face_tags()

    while faces:
        f = random.choice(faces)  # Prend une face aléatoirement
        points_candidats = get_cutable_pts(f)  # Récupère les points candidats pour être coupé
        p = random.choice(points_candidats)  # Récupère un point
        c = random.choice(can_cut(p, f))  # Récupère une découpe possible

        cut(p, f, c)  # Fait la découpe

        faces = get_all_non_square_faces()

def direction_choice(p, f_tag):
    possibilite_coupes = can_cut(p, f_tag)
    taille = get_taille(f_tag)
    if taille[0] < taille[1]:
        if 3 in possibilite_coupes:
            return 3
        if 4 in possibilite_coupes:
            return 4
    else:
        if 1 in possibilite_coupes:
            return 1
        if 2 in possibilite_coupes:
            return 2

    return possibilite_coupes[0]

def algo():
    # Récupère la surface initiale
    faces = get_face_tags()

    while faces:
        f = faces[0]  # Prend une face aléatoirement
        points_candidats = get_cutable_pts(f)  # Récupère les points candidats pour être coupé
        p = points_candidats[0]  # Récupère un point
        c = direction_choice(p, f)  # Récupère une découpe possible

        cut(p, f, c)  # Fait la découpe

        faces = get_all_non_square_faces()

def Q_Learning_train_random(ScoreMin, NBTest, shape, gamma):

    Data = {}
    RefImplemente = {}
    #random.seed(2)
    nbbcl = 0
    q = Query()
    AncienneRef = ""

    alpha = 1

    for s in shape:
        for nb in range(NBTest):
            gmsh.clear()
            print("Clear")

            file_name = s + ".txt"
            f = open(file_name, "r")
            ps = create_shape(f)
            f.close()

            c = center( get_face_tags()[0])

            ListeAction = {}
            Bonus = {}

            Face = get_all_non_square_faces()

            print("Début preocess: ", nb)

            while len(Face) != 0: #and nbbcl != 10:
                nbbcl += 1
                i = 0

                #print(len(get_face_tags()))

                #séléction de la face parmis celle qui n'on pas 4 points en fonction de Data

                #séléction random
                if len(Face) == 1:
                    r = 0
                else:
                    r = random.randint(0, (len(Face) - 1))

                points = q.get_corners(Face[r])
                f_tag = Face[r]

                print(len(Face), " face a découper")
                print("selection face: ", f_tag)

                #selection aléatoire du point

                PointValide = []
                PoidsMax = -999999

                for p in points:
                    if len(can_cut(p, f_tag)) != 0:
                        PointValide.append(p)

                        ref = get_ref_point(p, Face, f_tag, c)

                        if not ref in RefImplemente:
                            RefImplemente[ref] = False

                        if RefImplemente[ref]:
                            for poids in Data[ref]:
                                if poids > PoidsMax:
                                    PoidsMax = poids


                if AncienneRef != "" and PoidsMax != -999999:
                    if not AncienneRef in Bonus:
                        Bonus[AncienneRef] = [[], [], [], []]

                    print("PoidsMax: ",PoidsMax)
                    Bonus[AncienneRef][DernierChoix].append(gamma * (PoidsMax - Data[AncienneRef][DernierChoix]))

                if len(PointValide) == 1:
                    r = 0
                else:
                    r = random.randint(0, len(PointValide) - 1)

                p = PointValide[r]

                print("selection point: ", p)

                ref = get_ref_point(p, Face, f_tag, c)

                if not ref in Data:
                    Data[ref] = [0,0,0,0]
                if not ref in ListeAction:
                    ListeAction[ref] = [0,0,0,0]

                #selection de la direction a cut

                CanCut = can_cut(p, f_tag)

                print(p, f_tag)

                if len(CanCut) == 1:
                    r = 0
                else:
                    r = random.randint(0, len(CanCut) - 1)



                print("cut: ", CanCut[r])
                print(CanCut)

                if ref == "1/[]/[2, 3]/":
                    return False

                cut(p, f_tag, CanCut[r])
                DernierChoix = CanCut[r] - 1
                ListeAction[ref][CanCut[r] - 1] +=1
                AncienneRef = ref
                Face = get_all_non_square_faces()

            if get_score() <= ScoreMin:
                gain = 1
                print("Gagner, ", get_score(), " < ", ScoreMin)
            else:
                gain = -1
                print("Perdu, ", get_score(), " > ", ScoreMin)

            for ref in ListeAction.keys():
                RefImplemente[ref] = True
                for i in range(4):
                    bonus = 0
                    if ref in Bonus:
                        for j in Bonus[ref][i]:
                            bonus += j
                    Data[ref][i] += alpha * (ListeAction[ref][i] * gain + bonus)
                    if alpha * (ListeAction[ref][i] * gain + bonus) != 0:
                        print("récompense de: ", alpha * (ListeAction[ref][i] * gain + bonus))

            print("Fin preocess: ", nb, " shape_", s)
        print("Fin calcul: ", s)
    return Data

def Q_Learning_execute(Data, shape):
    gmsh.clear()
    file_name = shape + ".txt"
    f = open(file_name, "r")
    ps = create_shape(f)
    f.close()
    q = Query()

    c = center(get_face_tags()[0])

    Face = get_all_non_square_faces()
    nb = 0

    while len(Face) != 0 and nb < 100:  # and nbbcl != 10:
        # séléction de la face parmis celle qui n'on pas 4 points en fonction de Data
        nb +=1
        Max = []

        for f in Face:

            points = q.get_corners(f)
            PointValide = []
            MaxSomme = [-1, -9999, ""]

            for p in points:
                if len(can_cut(p, f)) != 0:
                    PointValide.append(p)
            for p in PointValide:
                ref = get_ref_point(p, Face, f, c)
                if ref in Data:
                    somme = Data[ref][0] + Data[ref][1] + Data[ref][2] + Data[ref][3]
                    if somme > MaxSomme[1]:
                        MaxSomme[0] = p
                        MaxSomme[1] = somme
                        MaxSomme[2] = ref
            Max.append(MaxSomme)

        PoidsMax = -9999
        PMax = -1
        RefMax = ""
        FaceMax = 0

        for i in range(len(Face)):
            if PoidsMax < Max[i][1]:
                PMax =  Max[i][0]
                PoidsMax =  Max[i][1]
                RefMax =  Max[i][2]
                FaceMax = Face[i]

        if PMax == -1:
            # séléction random
            if len(Face) == 1:
                r = 0
            else:
                r = random.randint(0, (len(Face) - 1))

            FaceMax = Face[r]

            points = q.get_corners(Face[r])
            PointValide = []

            for p in points:
                if can_cut(p, FaceMax) != []:
                    PointValide.append(p)

            print("séléction aléatoire")

            if len(PointValide) == 1:
                PMax = PointValide[0]
                RefMax = get_ref_point(p, Face, FaceMax, c)
            else:
                print(PointValide)

                PMax = PointValide[random.randint(0, len(PointValide) - 1)]
                RefMax = get_ref_point(p, Face, FaceMax, c)

        print(len(Face), " face a découper")
        print("selection face: ", FaceMax)

        # selection aléatoire du point

        print("selection point: ", PMax)

        # selection de la direction a cut

        if not RefMax in Data:
            CanCut = can_cut(PMax, FaceMax)
            print(CanCut)
            dir = CanCut[random.randint(0, len(CanCut) - 1)]
            print("séléction aléatoire de la direction")
        else:
            max = -9999
            dir = -1
            print(Data[RefMax])
            print(RefMax)
            for i in range(4):
                if Data[RefMax][i] > max and Data[RefMax][i] != 0:
                    max = Data[RefMax][i]
                    dir = i+1
            if dir == -1:
                CanCut = can_cut(PMax, FaceMax)
                print("cut aléatoire ", CanCut)
                dir = CanCut[random.randint(0, len(CanCut) - 1)]

        cut(PMax, FaceMax, dir)

        print("cut: ", dir)

        Face = get_all_non_square_faces()

    print("Fin Execute, score =", get_score())

    if nb == 100:
        return False
    return(get_score())

def Search_Param(shape):

    #cherche le RatioMin idéal
    NBTest = 10
    Gamma= 0.1
    RatioMin = 1.8

    ScoreMin = 9999

    """for i in range(30):
        score = 0
        Data = Q_Learning_train_random(i/10, NBTest, shape, 0.1)
        for j in range(31, 41):
            score += Q_Learning_execute(Data, "shape_" + str(j))
        score = score/10
        print("Score moyen pour NBTest = ", i/10, ": ", score)

        if score <= ScoreMin:
            RatioMin = i/10
            ScoreMin = score
        print("RatioTest: ", RatioMin)

    print("Selection RatioMin = ", RatioMin)"""

    ScoreMin = 99999
    for i in range(10):
        score = 0
        Data = Q_Learning_train_random(RatioMin, NBTest, shape, i/100)
        for j in range(31, 41):
            score += Q_Learning_execute(Data, "shape_" + str(j))
        score = score/10
        print("Score moyen pour Gamma = ", i/100, ": ", score)

        if score <= ScoreMin:
            Gamma = i/100
            ScoreMin = score
        print("RatioTest: ", Gamma)

    print("Selection Gamma = ", Gamma)

    ScoreMin = 9999

    for i in range(5):
        score = 0
        Data = Q_Learning_train_random(RatioMin, i*10, shape, Gamma)
        for j in range(31, 41):
            score += Q_Learning_execute(Data, "shape_" + str(j))
        score = score / 10
        print("Score moyen pour NBTest = ", i*10, ": ", score)

        if score <= ScoreMin:
            NBTest = i*10
            ScoreMin = score
        print("NBTest: ", NBTest)

    print("Selection NBTest = ", NBTest)

    return [RatioMin, Gamma, NBTest]

def Q_Learning_train_Algo(ScoreMin, NBTest, shape, gamma):

    Data = {}
    RefImplemente = {}
    #random.seed(2)
    gain = 10
    q = Query()
    AncienneRef = ""
    alpha = 1
    for s in shape:
        for nb in range(NBTest):
            gmsh.clear()
            print("Clear")
            file_name = s + ".txt"
            f = open(file_name, "r")
            ps = create_shape(f)
            f.close()

            centre = center( get_face_tags()[0])

            ListeAction = {}
            Bonus = {}

            # Récupère la surface initiale
            faces = get_face_tags()

            while faces:
                f = faces[0]  # selection d'une face
                points_candidats = get_cutable_pts(f)  # Récupère les points candidats pour être coupé

                PoidsMax = -999999

                for p in points_candidats:
                    ref = get_ref_point_deterministe(p, faces, centre)

                    if not ref in RefImplemente:
                        RefImplemente[ref] = False

                    if RefImplemente[ref]:
                        for poids in Data[ref]:
                            if poids > PoidsMax:
                                PoidsMax = poids


                if AncienneRef != "" and PoidsMax != -999999:
                    if not AncienneRef in Bonus:
                        Bonus[AncienneRef] = [[], [], [], []]

                        print(DernierChoix)
                        Bonus[AncienneRef][DernierChoix].append(gamma * (PoidsMax - Data[AncienneRef][DernierChoix]))

                points_candidats = get_cutable_pts(f)
                p = points_candidats[0]  # Récupère un point
                c = direction_choice(p, f)  # Récupère une découpe possible

                ref = get_ref_point_deterministe(p, get_all_non_square_faces(), centre)

                if not ref in Data:
                    Data[ref] = [0, 0, 0, 0]
                if not ref in ListeAction:
                    print("add action ref")
                    ListeAction[ref] = [0, 0, 0, 0]

                cut(p, f, c)  # Fait la découpe

                DernierChoix = c - 1
                ListeAction[ref][c - 1] += 1
                AncienneRef = ref

                faces = get_all_non_square_faces()

        if get_score() <= ScoreMin:
            gain = 1
            print("Gagner, ", get_score(), " < ", ScoreMin)
        else:
            gain = -1
            print("Perdu, ", get_score(), " > ", ScoreMin)

        if len(ListeAction) != 0:
            for ref in ListeAction.keys():
                RefImplemente[ref] = True
                for i in range(4):
                    bonus = 0
                    if ref in Bonus:
                        for j in Bonus[ref][i]:
                            bonus += j
                    Data[ref][i] += alpha * (ListeAction[ref][i] * gain + bonus)
                    if alpha * (ListeAction[ref][i] * gain + bonus) != 0:
                        print("récompense de: ", alpha * (ListeAction[ref][i] * gain + bonus))

        print("Fin preocess: ", nb, " shape_", s)

    return Data

def Q_Learning_execute_Deterministe(Data, shape):
    gmsh.clear()
    file_name = shape + ".txt"
    f = open(file_name, "r")
    ps = create_shape(f)
    f.close()
    q = Query()

    c = center(get_face_tags()[0])

    Face = get_all_non_square_faces()

    while len(Face) != 0:  # and nbbcl != 10:
        # séléction de la face parmis celle qui n'on pas 4 points en fonction de Data

        f_tag = Face[0]  # Prend une face aléatoirement

        points = q.get_corners(f_tag)

        PointValide = []

        for p in points:
            if len(can_cut(p, f_tag)) != 0:
                PointValide.append(p)

        if len(PointValide) == 1:
            p = PointValide[0]
            ref = get_ref_point_deterministe(p, Face, c)
            print("1 point valide")
        else:
            #selectionne le point dont la somme des poids est la plus élevé
            MaxSomme = [-1, 0, ""]
            for p in PointValide:
                ref = get_ref_point_deterministe(p, Face, c)
                if ref in Data:
                    somme = Data[ref][0] + Data[ref][1] + Data[ref][2] + Data[ref][3]
                    #print("somme ", somme, " point", p)
                    if somme > MaxSomme[1]:
                        MaxSomme[0] = p
                        MaxSomme[1] = somme
                        MaxSomme[2] = ref
                """else:
                    #print("ref inconu: ", ref)"""
            if MaxSomme[0] == -1:

                points_candidats = get_cutable_pts(f_tag)  # Récupère les points candidats pour être coupé
                p = points_candidats[0]  # Récupère un point

                ref = get_ref_point_deterministe(p, Face, c)
            else:
                p = MaxSomme[0]
                ref = MaxSomme[2]

        print("selection point: ", p)
        # selection de la direction a cut

        if not ref in Data:
            CanCut = can_cut(p, f_tag)

            dir = direction_choice(p, f_tag)
            print("séléction aléatoire de la direction")
        else:
            max = 0
            dir = -1
            for i in range(4):
                if Data[ref][i] > max:
                    max = Data[ref][i]
                    dir = i+1
            if dir == -1:
                CanCut = can_cut(p, f_tag)
                dir = direction_choice(p, f_tag)

        cut(p, f_tag, dir)

        #print("cut: ", dir)

        Face = get_all_non_square_faces()

    #print("Fin Execute, score =", get_score())
    return(get_score())

def Search_Param_Deterministe(shape):

    #cherche le RatioMin idéal
    NBTest = 5
    Gamma= 0.1
    RatioMin = 2.9

    ScoreMin = 9999

    for i in range(30):
        score = 0
        Data = Q_Learning_train_Algo(i/10, NBTest, shape, Gamma)
        for j in range(31, 41):
            score += Q_Learning_execute_Deterministe(Data, "shape_" + str(j))
        score = score/10
        print("Score moyen pour NBTest = ", i/10, ": ", score)

        if score <= ScoreMin:
            RatioMin = i/10
            ScoreMin = score
        print("RatioTest: ", RatioMin)

    print("Selection RatioMin = ", RatioMin)

    ScoreMin = 99999
    for i in range(30):
        score = 0
        Data = Q_Learning_train_Algo(RatioMin, NBTest, shape, i/10)
        for j in range(31, 41):
            score += Q_Learning_execute_Deterministe(Data, "shape_" + str(j))
        score = score/10
        print("Score moyen pour NBTest = ", i/10, ": ", score)

        if score <= ScoreMin:
            Gamma = i/10
            ScoreMin = score
        print("RatioTest: ", Gamma)

    print("Selection Gamma = ", Gamma)

    ScoreMin = 9999

    for i in range(1,10):
        score = 0
        Data = Q_Learning_train_Algo(RatioMin, i, shape, Gamma)
        for j in range(31, 41):
            score += Q_Learning_execute_Deterministe(Data, "shape_" + str(j))
        score = score / 10
        print("Score moyen pour NBTest = ", i*10, ": ", score)

        if score <= ScoreMin:
            NBTest = i*10
            ScoreMin = score
        print("NBTest: ", NBTest)

    print("Selection NBTest = ", NBTest)

    return [RatioMin, Gamma, NBTest]

#######################################################

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # first, we initialize the gmsh environment
    initialize()

    shape = []
    for i in range(1, 41):
        shape.append("shape_" + str(i))

    # ======================================================
    #   Recherche paramètre optimaux
    # ======================================================

    """start = now = datetime.datetime.now()

    print(Search_Param_Deterministe(shape))

    end = now = datetime.datetime.now()

    print("la recherche a duré ", end - start)"""

    #======================================================
    #   Parti aprentisage
    #======================================================

    score = 2

    start = now = datetime.datetime.now()


    Data = Q_Learning_train_random(1.5 , 100, shape,0.02)
    #Data = Q_Learning_train_Algo(1.8, 5, shape, 0.46)

    #print(Search_Param(shape))

    end = now = datetime.datetime.now()


    # ======================================================
    #   Parti utilisation
    # ======================================================

    """Data = dict(enumerate(np.load("Data\data_" + str(1) + ".npy", allow_pickle='TRUE').flatten(), 1))[1]
    print(Data)
    score = 0"""

    """for i in range(31, 42):
    
        score += Q_Learning_execute(Data, "shape_" + str(i) )"""
    """score = Q_Learning_execute(Data, "shape_11")
    print("Score: ", score)

    print(len(Data))
    print("la recherche a duré ", end - start)
    remesh()

    i = 1"""
    """while os.path.isfile("Data\data_" + str(i) + ".npy"):
        i += 1"""

    np.save("Data\data_" + str(i) + ".npy", Data)
    print("la recherche a duré ", end - start)
    gmsh.write("mesh_gmsh.vtk")
    finalize()
