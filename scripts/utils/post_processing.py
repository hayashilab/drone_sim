import numpy as np
from tabulate import tabulate

def haversine(lat1, lon1, lat2, lon2):

    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    """Haversine formula"""

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    r = 6371000  
    return c * r

def bearing(lat1, lon1, lat2, lon2):

    """Find direction of the distance error"""

    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    x = np.sin(lon2 - lon1) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(lon2 - lon1)
    initial_bearing = np.arctan2(x, y)

    initial_bearing = np.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    return bearing

def calculate_errors(reference, measured):
    distance_errors = []
    bearings = []
    nearest_model = []
    for measured_key, measured_value in measured.items():
        nearest_meas_value, nearest_model_name, min_distance = find_nearest_point(measured_value, reference)
        nearest_model.append(nearest_model_name)
        distance_errors.append(min_distance)
        bearings.append(bearing(measured_value['lat'], measured_value['lon'], nearest_meas_value['lat'], nearest_meas_value['lon']))

    """find MAE MSE"""

    mae = np.mean(np.abs(distance_errors))
    mse = np.mean(np.square(distance_errors))
    rmse = np.sqrt(mse) 
    

    return {
        'nearest_model':nearest_model,
        'distance_errors': distance_errors,
        'directional_errors': bearings,
        'MAE': mae,
        'MSE': mse,
        'RMSE': rmse,
    }

def find_nearest_point(measured, ref_point):
    min_distance = float('inf')
    nearest_point = None
    for m_key, m_point in ref_point.items():
        distance = haversine(measured['lat'], measured['lon'], m_point['lat'], m_point['lon'])
        if distance < min_distance:
            min_distance = distance
            nearest_point = m_point
            nearest_model = m_key
    return nearest_point, nearest_model, min_distance

def check_duplicate(list):
    index = 0
    count = 1
    count_list = []
    for i in range(1,len(list)):
        if list[i] == list[index]: count+=1
        else: 
            if count > 1: count_list.append([list[index],count])
            index = i
            count = 1

        if i+1 == len(list) and count > 1:
            count_list.append([list[index],count])
            break
    return count_list

# def calculate_errors(reference, measured):
#     distance_errors = []
#     bearings = []
     
#     for ref_key, ref_value in reference.items():
#         nearest_meas_value, min_distance = find_nearest_point(ref_value, measured)
#         distance_errors.append(min_distance)
#         bearings.append(bearing(ref_value['lat'], ref_value['lon'], nearest_meas_value['lat'], nearest_meas_value['lon']))

#     """find MAE MSE"""

#     mae = np.mean(np.abs(distance_errors))
#     mse = np.mean(np.square(distance_errors))

#     return {
#         'distance_errors': distance_errors,
#         'directional_errors': bearings,
#         'MAE': mae,
#         'MSE': mse
#     }

# def find_nearest_point(ref_point, measured):
#     min_distance = float('inf')
#     nearest_point = None
#     for m_key, m_point in measured.items():
#         distance = haversine(ref_point['lat'], ref_point['lon'], m_point['lat'], m_point['lon'])
#         if distance < min_distance:
#             min_distance = distance
#             nearest_point = m_point
#     return nearest_point, min_distance

"""reference = {'beer_0': {'lat': 33.65514081713295, 'lon': 130.67404008500068}, 'beer_1': {'lat': 33.655233461831926, 'lon': 130.67397059354153}, 'beer_2': {'lat': 33.655091771253474, 'lon': 130.674033522703}, 'beer_3': {'lat': 33.65505249429643, 'lon': 130.67405232681605}, 'beer_4': {'lat': 33.655187967720195, 'lon': 130.67382154741688}, 'beer_5': {'lat': 33.65528087159476, 'lon': 130.67399675790608}, 'beer_6': {'lat': 33.655084354220016, 'lon': 130.67385092717686}, 'beer_7': {'lat': 33.65508383738875, 'lon': 130.67396239327294}, 'beer_8': {'lat': 33.655216793274676, 'lon': 130.67386338018935}, 'beer_9': {'lat': 33.65512151180353, 'lon': 130.6739940619967}, 'beer_10': {'lat': 33.65513292901026, 'lon': 130.67408011518418}, 'beer_11': {'lat': 33.65508718495883, 'lon': 130.67403947825395}, 'beer_12': {'lat': 33.65518504411845, 'lon': 130.67389717503448}, 'beer_13': {'lat': 33.65517042086282, 'lon': 130.67386550362335}, 'beer_14': {'lat': 33.655093112297656, 'lon': 130.67393573359013}, 'beer_15': {'lat': 33.65530322525785, 'lon': 130.6740052107076}, 'beer_16': {'lat': 33.655254975628765, 'lon': 130.67398601423835}, 'beer_17': {'lat': 33.655047505571694, 'lon': 130.6739845865853}, 'beer_18': {'lat': 33.655058277645026, 'lon': 130.6739172163681}, 'beer_19': {'lat': 33.655107492605886, 'lon': 130.67405219663976}, 'beer_20': {'lat': 33.65513293190712, 'lon': 130.67396506267963}, 'beer_21': {'lat': 33.6550305973866, 'lon': 130.67380135147303}, 'beer_22': {'lat': 33.65507407221268, 'lon': 130.67379164755337}, 'beer_23': {'lat': 33.655036859281836, 'lon': 130.67385856420358}, 'beer_24': {'lat': 33.65505088556544, 'lon': 130.6739258804351}, 'beer_25': {'lat': 33.65519460143808, 'lon': 130.6738750549872}, 'beer_26': {'lat': 33.655035342670274, 'lon': 130.6737950593303}, 'beer_27': {'lat': 33.65522857091619, 'lon': 130.67406565776545}, 'beer_28': {'lat': 33.655330036060164, 'lon': 130.6740017152437}, 'beer_29': {'lat': 33.655263309376174, 'lon': 130.6739470813793}, 'bowl_0': {'lat': 33.655039614118564, 'lon': 130.6739145029637}, 'bowl_1': {'lat': 33.655088386424666, 'lon': 130.67389901281095}, 'bowl_2': {'lat': 33.655281904679285, 'lon': 130.67399561011584}, 'bowl_3': {'lat': 33.65532398356847, 'lon': 130.6738651239606}, 'bowl_4': {'lat': 33.65527485553514, 'lon': 130.67377767050957}, 'bowl_5': {'lat': 33.655302503061385, 'lon': 130.67393863412698}, 'bowl_6': {'lat': 33.65529329498045, 'lon': 130.6740329940244}, 'bowl_7': {'lat': 33.65527441493363, 'lon': 130.6738156428721}, 'bowl_8': {'lat': 33.65511788436489, 'lon': 130.67391122946856}, 'bowl_9': {'lat': 33.65522793165086, 'lon': 130.67397230540692}, 'bowl_10': {'lat': 33.65503480476647, 'lon': 130.67401036431028}, 'bowl_11': {'lat': 33.655157733802305, 'lon': 130.67404534667955}, 'bowl_12': {'lat': 33.65526746909081, 'lon': 130.67386476929218}, 'bowl_13': {'lat': 33.65503703222753, 'lon': 130.6737996378266}, 'bowl_14': {'lat': 33.65522252251996, 'lon': 130.67400088992252}, 'bowl_15': {'lat': 33.65506038759586, 'lon': 130.67382308913787}, 'bowl_16': {'lat': 33.65512770594614, 'lon': 130.67392169054426}, 'bowl_17': {'lat': 33.65521146177456, 'lon': 130.67380459112033}, 'bowl_18': {'lat': 33.65507278813894, 'lon': 130.6739393835543}, 'bowl_19': {'lat': 33.655199947848246, 'lon': 130.67402380326723}, 'bowl_20': {'lat': 33.65529497808239, 'lon': 130.67401160284885}, 'bowl_21': {'lat': 33.65516602912645, 'lon': 130.674066345261}, 'bowl_22': {'lat': 33.65526534338005, 'lon': 130.67404296634703}, 'bowl_23': {'lat': 33.65521016123915, 'lon': 130.67393334527333}, 'bowl_24': {'lat': 33.65517893012256, 'lon': 130.67385694368488}, 'bowl_25': {'lat': 33.65523210333191, 'lon': 130.67378583843615}, 'bowl_26': {'lat': 33.65525688537802, 'lon': 130.67395892417136}, 'bowl_27': {'lat': 33.65517820938372, 'lon': 130.67399962190277}, 'bowl_28': {'lat': 33.6552171467916, 'lon': 130.67391881042184}, 'bowl_29': {'lat': 33.655205156425104, 'lon': 130.67405940637738}}

measured = {112: {'lat': 33.65527585483495, 'lon': 130.67381421796293}, 114: {'lat': 33.655273818211214, 'lon': 130.6737765465259}, 118: {'lat': 33.65523112472518, 'lon': 130.67378587488423}, 119: {'lat': 33.65521151891625, 'lon': 130.6738063453665}, 120: {'lat': 33.655210880451975, 'lon': 130.67380380841837}, 121: {'lat': 33.655190240600554, 'lon': 130.67382159601925}, 194: {'lat': 33.65503403727908, 'lon': 130.67379484228582}, 185: {'lat': 33.65507383043555, 'lon': 130.67379159359106}, 226: {'lat': 33.65507264990907, 'lon': 130.67379051287307}, 193: {'lat': 33.65503541140572, 'lon': 130.6737988045964}, 225: {'lat': 33.65503534974757, 'lon': 130.67379893303078}, 192: {'lat': 33.65502966932265, 'lon': 130.67380249386076}, 128: {'lat': 33.65503871556321, 'lon': 130.67380140498582}, 129: {'lat': 33.65504085894762, 'lon': 130.67380320120415}, 215: {'lat': 33.65505976036323, 'lon': 130.67382119989057}, 224: {'lat': 33.6550596594147, 'lon': 130.67382314112288}, 247: {'lat': 33.6550856257205, 'lon': 130.67385222927223}, 262: {'lat': 33.65517003562887, 'lon': 130.67386679962462}, 282: {'lat': 33.655177416029076, 'lon': 130.67385723475135}, 281: {'lat': 33.655193502335365, 'lon': 130.6738763379181}, 283: {'lat': 33.655217378873, 'lon': 130.6738645366759}, 306: {'lat': 33.65526573932146, 'lon': 130.6738653710055}, 331: {'lat': 33.65532307571274, 'lon': 130.67386168742604}, 352: {'lat': 33.65530451193486, 'lon': 130.6739376290146}, 343: {'lat': 33.65530251501665, 'lon': 130.67393781457585}, 354: {'lat': 33.65526458063498, 'lon': 130.67394591266537}, 355: {'lat': 33.65525773240214, 'lon': 130.6739561545371}, 356: {'lat': 33.655218357738555, 'lon': 130.67391722133118}, 357: {'lat': 33.65521089880331, 'lon': 130.67393371822666}, 358: {'lat': 33.65521078920799, 'lon': 130.67393371609984}, 363: {'lat': 33.65521172710168, 'lon': 130.67393334473385}, 367: {'lat': 33.65521188834981, 'lon': 130.67393190820076}, 369: {'lat': 33.655185644694086, 'lon': 130.6738955884757}, 373: {'lat': 33.655128996639704, 'lon': 130.6739213968187}, 374: {'lat': 33.6551286572118, 'lon': 130.67392010141043}, 375: {'lat': 33.65511915224758, 'lon': 130.6739095144061}, 379: {'lat': 33.6550889797247, 'lon': 130.6738976069089}, 382: {'lat': 33.655088241577616, 'lon': 130.67389689982969}, 393: {'lat': 33.65505963625707, 'lon': 130.67391727896137}, 395: {'lat': 33.6550601627303, 'lon': 130.67391656744428}, 390: {'lat': 33.655058979892345, 'lon': 130.67391740213628}, 392: {'lat': 33.65505918473921, 'lon': 130.67391729634207}, 396: {'lat': 33.65503989677146, 'lon': 130.67391368546035}, 436: {'lat': 33.6550586985562, 'lon': 130.67391756143033}, 394: {'lat': 33.655054597020055, 'lon': 130.67392600873438}, 411: {'lat': 33.65505078190254, 'lon': 130.67392642269493}, 460: {'lat': 33.65507148320238, 'lon': 130.67393516600188}, 465: {'lat': 33.65507130999201, 'lon': 130.67393651386902}, 387: {'lat': 33.6550728384092, 'lon': 130.67393904305922}, 388: {'lat': 33.655074099283254, 'lon': 130.67393895148976}, 389: {'lat': 33.65507425312106, 'lon': 130.6739377563022}, 443: {'lat': 33.65507292752398, 'lon': 130.67393824729422}, 466: {'lat': 33.655083751038156, 'lon': 130.67396197218068}, 468: {'lat': 33.65504683014943, 'lon': 130.6739838772801}, 505: {'lat': 33.65504854716264, 'lon': 130.67398551324573}, 517: {'lat': 33.655091597926656, 'lon': 130.67403499854692}, 518: {'lat': 33.65512027995306, 'lon': 130.6739955909275}, 519: {'lat': 33.65513198344322, 'lon': 130.67396573917}, 520: {'lat': 33.65513226945459, 'lon': 130.6739656555155}, 523: {'lat': 33.65513252395687, 'lon': 130.673967029398}, 533: {'lat': 33.655175995202725, 'lon': 130.67400039991705}, 524: {'lat': 33.65517727076237, 'lon': 130.67400005305515}, 538: {'lat': 33.65519887855195, 'lon': 130.67402493641208}, 539: {'lat': 33.65519772128949, 'lon': 130.67402472700144}, 540: {'lat': 33.655221265647555, 'lon': 130.67400179199603}, 541: {'lat': 33.65522562109886, 'lon': 130.67397328946404}, 542: {'lat': 33.655231391455544, 'lon': 130.6739724747807}, 543: {'lat': 33.655254046974974, 'lon': 130.67398763371747}, 545: {'lat': 33.655254858418076, 'lon': 130.673960145393}, 544: {'lat': 33.655255927941205, 'lon': 130.6739595712924}, 602: {'lat': 33.65527883354571, 'lon': 130.67399758276017}, 617: {'lat': 33.65527885120065, 'lon': 130.67399615177897}, 667: {'lat': 33.65530300113417, 'lon': 130.67400381467831}, 559: {'lat': 33.65530280071401, 'lon': 130.67400701497155}, 560: {'lat': 33.6553024202547, 'lon': 130.67400704283975}, 574: {'lat': 33.655301696341645, 'lon': 130.67400732154562}, 604: {'lat': 33.65530229991576, 'lon': 130.67400724310076}, 666: {'lat': 33.655293353694915, 'lon': 130.67401115730166}, 550: {'lat': 33.65529407655139, 'lon': 130.67401262381298}, 558: {'lat': 33.655293069447296, 'lon': 130.67401237660675}, 601: {'lat': 33.6552926795901, 'lon': 130.6740131637418}, 699: {'lat': 33.655293462482895, 'lon': 130.67403066196002}, 684: {'lat': 33.655291681232676, 'lon': 130.67402969867754}, 710: {'lat': 33.6552925577221, 'lon': 130.6740306008199}, 709: {'lat': 33.655265174976726, 'lon': 130.67404080609194}, 712: {'lat': 33.65523048489312, 'lon': 130.67406438649402}, 713: {'lat': 33.6552036723408, 'lon': 130.6740576021259}, 719: {'lat': 33.65516687433893, 'lon': 130.67406421226352}, 720: {'lat': 33.65515966862833, 'lon': 130.67404248252845}, 721: {'lat': 33.65514223955417, 'lon': 130.67403775037974}, 722: {'lat': 33.6551334581533, 'lon': 130.67407948017672}, 723: {'lat': 33.65513413439519, 'lon': 130.67407940400753}, 724: {'lat': 33.65513412204446, 'lon': 130.6740789925737}, 726: {'lat': 33.65510922576949, 'lon': 130.67405006329048}, 734: {'lat': 33.655056059234866, 'lon': 130.67405038213695}}


errors = calculate_errors(reference, measured)"""


"""print("Distance Errors (meters):", errors['distance_errors'])
print("Directional Errors (degrees):", errors['directional_errors'])
print("MAE (meters):", errors['MAE'])
print("MSE (meters squared):", errors['MSE'])
data = [{
    "Metric": "Distance Errors (meters)",
    "Values": str(errors['distance_errors'])
}, {
    "Metric": "Directional Errors (degrees)",
    "Values": str(errors['directional_errors'])
}, {
    "Metric": "MAE (meters)",
    "Values": str(errors['MAE'])
}, {
    "Metric": "MSE (meters squared)",
    "Values": str(errors['MSE'])
}]"""



"""data = []
for i,j  in enumerate(reference):
    # for j in range(0,1):
        #print(errors['distance_errors'][i], errors['directional_errors'][i])
        data.append([j,errors['distance_errors'][i], errors['directional_errors'][i]])


col_names = ["Object", "Distance error", "Direction error"]
print(tabulate(data, headers=col_names, tablefmt="fancy_grid", showindex="always"))
print("MAE (meters):", errors['MAE'])
print("MSE (meters squared):", errors['MSE'])"""