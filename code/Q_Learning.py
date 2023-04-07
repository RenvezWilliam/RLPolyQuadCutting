import random
from math import *
from numpy import zeros,array
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

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
        print("Face ", f_tag,", center ", center(f_tag),", and area ", area(f_tag))
        print(" --> curves ", q.get_curves(f_tag))
        print(" --> points ", q.get_corners(f_tag))


#===Mes ajouts===#

def glouton():
    all_faces_has_four_points = True
    for f_tag in get_face_tags():
        q = Query()
        points = q.get_corners(f_tag)

        if points != 4:

            all_faces_has_four_points = False

            for p in points:
                for i in range(4):
                    j = i + 1
                    if can_cut(p, f_tag, j):
                        print("success")
                        print(p, f_tag, j)

                        cut(p, f_tag, j)
                        break
                    break
                break

    if not all_faces_has_four_points:
        glouton()

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

def can_cut(point, f_tag):
    # En utilisant le point et le f_tag, on cherche à voir s'il y a une curve adjaçant au point dans la même direction.
    """dir = 1 = nord | 2 = sud | 3 = est | 4 = ouest"""
    q = Query()
    all_curves = q.get_curves(f_tag)
    c_point = point_coordinate(point)
    returned_value = []
    value = []

    for c in all_curves:
        pts = get_end_points(c)
        cpts1 = point_coordinate(pts[0])
        cpts2 = point_coordinate(pts[1])

        #   Nord
        if cpts1[1] > c_point[1] and cpts2[1] > c_point[1] and (
                cpts1[0] >= c_point[0] >= cpts2[0] or cpts2[0] >= c_point[0] >= cpts1[0]):
            value.append(1)

        #   Sud
        if cpts1[1] < c_point[1] and cpts2[1] < c_point[1] and (
                cpts1[0] >= c_point[0] >= cpts2[0] or cpts2[0] >= c_point[0] >= cpts1[0]):
            value.append(2)

        #   Est
        if cpts1[0] > c_point[0] and cpts2[0] > c_point[0] and (
                cpts1[1] >= c_point[1] >= cpts2[1] or cpts2[1] >= c_point[1] >= cpts1[1]):
            value.append(3)

        #   Ouest
        if cpts1[0] < c_point[0] and cpts2[0] < c_point[0] and (
                cpts1[1] >= c_point[1] >= cpts2[1] or cpts2[1] >= c_point[1] >= cpts1[1]):
            value.append(4)

    #   Suprimme les doublons

    for v in value:
        if not v in returned_value:
            returned_value.append(v)

    for c in all_curves:
        pts = get_end_points(c)
        if point == pts[0]:
            returned_value.remove(get_dir(point, pts[1]))
        elif point == pts[1]:
            returned_value.remove(get_dir(point, pts[0]))

    return returned_value

#######################################################

def get_x(point_tag):
    # renvoi la coordoné sur x d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return x

def get_y(point_tag):
    # renvoi la coordoné sur y d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return y

def get_z(point_tag):
    # renvoi la coordoné sur z d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return z

def get_dist(point_tag1, point_tag2):
    #renvoi la distance entre 2 points

    return sqrt((get_x(point_tag1)-get_x(point_tag2))**2 + (get_y(point_tag1)-get_y(point_tag2))**2)

def get_dir(point_tag1, point_tag2):
    #renvoi la direction d'un pooint 2 par rapport a un point 1
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

"""def get_nb_summit(f_tag):
    q = Query()
    points = q.get_corners(f_tag)
    curve = {}
    points2 = points

    for point in points:
        curve[get_x(point)] = 1
        curve[get_y(point)] = 1

    return len(curve)"""

def get_nb_summit(f_tag):

    q = Query()
    all_curves = q.get_curves(f_tag)
    points = q.get_corners(f_tag)

    for p1 in points:
        PointsValide = q.get_corners(f_tag)
        PointsValide.remove(p1)
        for p2 in PointsValide:
            PointsValide.remove(p2)
            if get_x(p2) == get_x(p1):
                for p3 in PointsValide:
                    if get_x(p2) == get_x(p1) == get_x(p3):
                        if get_x(p1) < get_x(p2) < get_x(p3) or get_x(p1) > get_x(p2) > get_x(p3) :
                            points.remove(p2)
                        elif get_x(p2) < get_x(p1) < get_x(p3) or get_x(p2) > get_x(p1) > get_x(p3) :
                            points.remove(p1)
                        elif get_x(p1) < get_x(p3) < get_x(p2) or get_x(p1) > get_x(p3) > get_x(p2) :
                            points.remove(p3)
            if get_y(p2) == get_y(p1):
                for p3 in PointsValide:
                    if get_y(p2) == get_y(p1) == get_y(p3):
                        if get_y(p1) < get_y(p2) < get_y(p3) or get_y(p1) > get_y(p2) > get_y(p3):
                            points.remove(p2)
                        elif get_y(p2) < get_y(p1) < get_y(p3) or get_y(p2) > get_y(p1) > get_y(p3):
                            points.remove(p1)
                        elif get_y(p1) < get_y(p3) < get_y(p2) or get_y(p1) > get_y(p3) > get_y(p2):
                            points.remove(p3)
    return len(points)

def is_on_a_face(point, direction, f_tag):

    q = Query()
    test = False
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

    point = Point((point_test))
    polygon = Polygon(coords)

    return polygon.covers(point), polygon.contains(point)

def Q_Learning_train(RatioMin, NBegal, gain, PoidInitial):

    Data = {}
    CopieData = Data
    nb = 0
    continuer = True

    NBProcess = 1

    #random.seed(2)

    nbbcl = 0

    while continuer:
        #print("CONTINUER")

        gmsh.clear()
        print("Clear")

        file_name = "shape.txt"
        f = open(file_name, "r")
        ps = create_shape(f)
        f.close()

        all_faces_has_not_four_points = True
        q = Query()

        ListeAction = {}
        ListeRef = []
        Face = []

        while all_faces_has_not_four_points: #and nbbcl != 10:
            nbbcl += 1
            all_faces_has_not_four_points = False
            Face.clear()
            i = 0
            #print(len(get_face_tags()))
            #séléction aléatoire de la face parmis celle qui n'on pas 4 points en fonction de Data

            for f_tag in get_face_tags():

                NBSummit = get_nb_summit(f_tag)
                print("face: ", f_tag, ", ", NBSummit, " points")
                if NBSummit > 4:
                    Face.append(f_tag)

            if not len(Face) == 0:
                all_faces_has_not_four_points = True

                #determine la ref corespondant a la situation actuel
                ref = "Face: "
                for j in range(len(Face)):
                    if not len(ref) == 0 :
                        ref = ref + "#"

                    point = q.get_corners(Face[j])
                    l = len(point)


                    for k in range(l):
                        if not len(ref) == 0:
                            ref = ref + "/"
                        if k == l-1:
                            ref = ref + str(get_dist(point[k], point[0]))
                        else:
                            ref = ref + str(get_dist(point[k], point[k+1]))

                if not ref in Data:
                        # initialisation des donnees

                    Data[ref] = zeros(len(Face), float)

                    for j in range(len(Face)):

                        Data[ref][j] = PoidInitial

                r = random.randint(0, 99)
                j = 0
                points = None
                sum = 0

                for d in Data[ref]:
                    sum = sum + d

                for k in range(len(Face)):
                    j = j + ((Data[ref][k]/sum) * 100)

                    if r < j:
                        points = q.get_corners(Face[k])
                        print("selection face: ", Face[k])
                        ListeAction[ref] = k
                        f_tag = Face[k]
                        ListeRef.append(ref)
                        break


                if len(points) == 0:
                    print("erreur lors de la selection de la face")

                #selection aléatoire du point en fonction de Data

                l = len(points)
                ref = "points: "

                for k in range(l):
                    if not len(ref) == 0:
                        ref = ref + "/"
                    if k == l-1:
                        ref = ref + str(get_dir(points[k], points[0])) + "/" + str(get_dist(points[k], points[0]))
                    else:
                        ref = ref + str(get_dir(points[k], points[k+1])) + "/" + str(get_dist(points[k], points[k+1]))

                # i apartien a lensemble des segement reliée a l'un des points A REMPLIR direction segment 1 / taille segment 1 / ... / direction segment i / taille segment i

                if not ref in Data:
                    #taille = len(points)
                    Data[ref] = []  #zeros(taille, float)
                    for i in range(len(points)):   #range(taille):
                        Data[ref].append(PoidInitial)
                print(ref)
                RefFace = ref

                ############################

                PointValide = []
                i = 0
                for p in points:
                    ref = "Direction :"

                    all_curves = get_edge_tags()

                    for c in all_curves:
                        pts = get_end_points(c)

                        if p == pts[0]:
                            ref = ref + str(get_dir(p, pts[1])) + "/" + str(get_dist(p, pts[1]))

                        if p == pts[1]:
                            ref = ref + str(get_dir(p, pts[0])) + "/" + str(get_dist(p, pts[0]))

                    # si le point ne peux pas être utilisé pour couper passe la broba de tomber dessu a 0 et choisi un autre point

                    if ref in Data:
                        for d in Data[ref]:
                            if d != 0:
                                PointValide.append(p)
                                continue

                        print("Proba: [", Data[ref][0], ", ", Data[ref][1], ", ", Data[ref][2], ", ", Data[ref][3], "]")

                    else:
                        PointValide.append(p)

                    if not p in PointValide:
                        Data[RefFace][i] = 0

                    i += 1

                ############################

                print("nombre de point valide: ", len(PointValide))

                if len(PointValide) >= 1:
                    p = None
                    #print("test")
                    if len(PointValide) == 1:
                        p = PointValide[0]
                        print("Séléction du point: ", p)
                        for i in range(len(points)):
                            if points[i] == p:
                                print(i)
                                break
                    else:
                        while not p in PointValide:
                            r = random.randint(0, 99)
                            j = 0
                            sum = 0

                            for d in Data[RefFace]:
                                sum = sum + d

                            print("somme: ", sum)

                            if sum != 0:
                                for i in range(len(points)):
                                    j = j + ((Data[RefFace][i]/sum) * 100)
                                    pb = Data[RefFace][i]

                                    #print(r, ", ", j)
                                    if r < j:
                                        p = points[i]
                                        print("Séléction du point: ", p)
                                        break
                    print(i)

                    ListeAction[RefFace] = i
                    ListeRef.append(RefFace)

                #   Selection de la Direction

                    ref = "Direction :"

                    all_curves = get_edge_tags()

                    for c in all_curves:
                        pts = get_end_points(c)

                        if p == pts[0]:
                            ref = ref + str(get_dir(p, pts[1])) + "/" + str(get_dist(p, pts[1]))

                        if p == pts[1]:
                            ref = ref + str(get_dir(p, pts[0])) + "/" + str(get_dist(p, pts[0]))

                print(ref)
                #direction segment 1 (0,1,2,3) / taille segment 1 / direction segment 2 (0,1,2,3) / taille segment 2

                if not ref in Data:
                    # initialisation des donnees 1 (north), 2 (south), 3 (east), 4 (west)
                    Data[ref] = [0.0, 0.0, 0.0, 0.0]

                    CanCut = can_cut(p, f_tag)
                    for var in CanCut:
                        Data[ref][var-1] = PoidInitial


                print("Nombre de cut possible: ", len(CanCut))
                # cut aléatoirement en fonction des proba renseigner dans Data
                r = random.randint(0, 99)
                j = 0

                print("Proba: [", Data[ref][0], ", ", Data[ref][1], ", ", Data[ref][2], ", ", Data[ref][3], "]")

                sum = 0

                for d in Data[ref]:
                    sum = sum + d

                if sum != 0:
                    for i in range(4):
                        j = j + ((Data[ref][i]/sum) * 100)
                        print(r, " / ", j)
                        if r < j:
                            #1 (north), 2 (south), 3 (east), 4 (west)

                            cut(p, f_tag, i+1)
                            print("cut ", i+1, ", face: ", f_tag)
                            ListeAction[ref] = i
                            ListeRef.append(ref)
                            break

        # calcul ratio résultat et attribution des récompenses
        ratio = 0

        for f_tag in get_face_tags():
            points = q.get_corners(f_tag)

            ratio = ratio + abs(get_dist(points[0], points[1]) - get_dist(points[1], points[2]))

        ratio = ratio / len(get_face_tags())

        for ref in ListeRef:
            if ratio < RatioMin:  # Bonus
                Data[ref][ListeAction[ref]] = Data[ref][ListeAction[ref]] + gain

            else:  # Malus
                #print(ref)
                Data[ref][ListeAction[ref]] = Data[ref][ListeAction[ref]] - gain


        #condition continuer
        continuer = False
        print("Fin process ", NBProcess)
        print(nb, ", ", NBegal)
        NBProcess+=1

        if CopieData == Data:
            nb+=1
            if nb == NBegal:
                continuer = False
                print("FIN")
        else:
            nb=0
        CopieData = Data

    return Data



def Q_Learning_Execute(Data):

    all_faces_has_not_four_points = True
    q = Query()

    while all_faces_has_not_four_points:

        all_faces_has_not_four_points = False
        Face = [len(get_face_tags())]
        i = 0

        # séléction aléatoire de la face parmis celle qui n'on pas 4 points en fonction de Data

        for f_tag in get_face_tags():

            points = q.get_corners(f_tag)

            if points != 4:
                Face[i] = f_tag
                i += 1
        if not len(Face) == 0:
            all_faces_has_not_four_points = True
        else:
            # determine la ref corespondant a la situation actuel
            for j in range(i):
                if not len(ref) == 0:
                    ref = ref + "#"

                point = q.get_corners(Face[j])
                l = len(point)

                for k in l:
                    if not len(ref) == 0:
                        ref = ref + "/"
                    if k == l:
                        ref = ref + str(get_dist(point[k], get_dist[0]))
                    else:
                        ref = ref + str(get_dist(point[k], get_dist[k + 1]))

            r = random.randint(0, 99)
            j = 0

            for k in range(i):
                j = j + (Data[ref][k] * 100)
                if r < j:
                    points = q.get_corners(Face[k])
                    break

            if len(points) == 0:
                print("erreur lors de la selection de la face")

            # selection aléatoire du point en fonction de Data

            l = len(points)

            for k in l:
                if not len(ref) == 0:
                    ref = ref + "/"
                if k == l:
                    ref = ref + str(get_dir(points[k], get_dist[0]), get_dist(points[k], get_dist[0]))
                else:
                    ref = ref + str(get_dir(points[k], get_dist[0]), get_dist(points[k], get_dist[k + 1]))

            # i apartien a lensemble des segement reliée a l'un des points A REMPLIR direction segment 1 / taille segment 1 / ... / direction segment i / taille segment i

            r = random.randint(0, 99)
            j = 0

            for i in range(len(points)):
                j = j + (Data[ref][i] * 100)
                if r < j:
                    p = points[i]
                    break

            if i + 1 == len(points):
                ref = ref + str(get_dir(p, points[i - 1])) + "/" + str(get_dist(p, points[i - 1])) \
                      + "/" + str(get_dir(p, points[0])) + "/" + str(get_dist(p, points[0]))
            elif i == 0:
                ref = ref + str(get_dir(p, points[len(points)])) + "/" + str(get_dist(p, points[len(points)])) \
                      + "/" + str(get_dir(p, points[i + 1])) + "/" + str(get_dist(p, points[i + 1]))
            else:
                ref = ref + str(get_dir(p, points[i - 1])) + "/" + str(get_dist(p, points[i - 1])) \
                      + "/" + str(get_dir(p, points[i + 1])) + "/" + str(get_dist(p, points[i + 1]))

            # direction segment 1 (0,1,2,3) / taille segment 1 / direction segment 2 (0,1,2,3) / taille segment 2

            # cut aléatoirement en fonction des proba renseigner dans Data
            r = random.randint(0, 99)
            j = 0

            for i in range(4):
                j = j + (Data[ref][i] * 100)
                if r < j:
                    cut(get_point_tags(p), f_tag, i)
                    break

#######################################################

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # first, we initialize the gmsh environment
    initialize()

    # ======================================================
    # Create a first shape by assembling rectangles
    # ======================================================

    """ r1 = create_rectangle(0, 0, 10, 10)
    r2 = create_rectangle(5, 5, 10, 3)
    r3 = create_rectangle(-5, 0, 7, 2)
    ps = fuse([r1, r2, r3])"""

    #RatioMin, NBegal, gain, PoidInitial

    Data = Q_Learning_train(3, 5, 10,50)

    #Q_Learning_Execute(Data)

    """q = Query()
    for f_tag in get_face_tags():

        for p in q.get_corners(f_tag):

            print("Point: ", p, " direction: ", can_cut(p, f_tag))"""



    gmsh.write("mesh_gmsh.vtk")
    finalize()

