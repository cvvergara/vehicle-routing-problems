/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef CONTAINERS_H
#define CONTAINERS_H

typedef struct Containers {
    int id;
    double lon;
    double lat;
    double demand;
    double open;
    double close;
    double service;
    double street_id;
} Containers;

static const Containers containers[] = {
// id, x, y, demand, open, close, service, street_id
{1546, -56.1559373201709, -34.9039843588323, 288, 1320, 1680, 2, -1},
{1553, -56.1560049669659, -34.9035142634204, 288, 1320, 1680, 2, -1},
{1565, -56.1566744569315, -34.9040343796724, 288, 1320, 1680, 2, -1},
{1572, -56.1574995689406, -34.9040910525684, 288, 1320, 1680, 2, -1},
{1574, -56.1573987356531, -34.9031891888916, 288, 1320, 1680, 2, -1},
{7355, -56.1562216903662, -34.9044197865767, 216, 1320, 1680, 2, -1},
{801, -56.155864138468, -34.9091080334181, 288, 1320, 1680, 2, -1},
{803, -56.1609972075495, -34.9069495850153, 216, 1320, 1680, 2, -1},
{804, -56.1611908972075, -34.9087841844239, 216, 1320, 1680, 2, -1},
{805, -56.1603481572987, -34.9070441922906, 288, 1320, 1680, 2, -1},
{806, -56.156099656173, -34.905254571393, 288, 1320, 1680, 2, -1},
{812, -56.1604831432847, -34.9068443634803, 288, 1320, 1680, 2, -1},
{1496, -56.1562096508294, -34.908337924196, 216, 1320, 1680, 2, -1},
{1497, -56.1571541752098, -34.9080288745468, 216, 1320, 1680, 2, -1},
{1500, -56.1607378657358, -34.9092331936079, 288, 1320, 1680, 2, -1},
{1503, -56.1597653451307, -34.9072378034786, 216, 1320, 1680, 2, -1},
{1505, -56.1612603166577, -34.9075842941059, 288, 1320, 1680, 2, -1},
{1507, -56.1625227702338, -34.9087130236676, 288, 1320, 1680, 2, -1},
{1510, -56.1618790099577, -34.9095898907302, 288, 1320, 1680, 2, -1},
{1511, -56.1620117437853, -34.9076456173097, 216, 1320, 1680, 2, -1},
{1512, -56.161421512152, -34.9066830729868, 288, 1320, 1680, 2, -1},
{1514, -56.1624179390047, -34.9083413074346, 216, 1320, 1680, 2, -1},
{1516, -56.1630998817183, -34.9067534456444, 288, 1320, 1680, 2, -1},
{1517, -56.1567552996726, -34.9086933169175, 216, 1320, 1680, 2, -1},
{1518, -56.1599614480861, -34.9085949743333, 288, 1320, 1680, 2, -1},
{1519, -56.1561667953488, -34.9094994694266, 216, 1320, 1680, 2, -1},
{1520, -56.156708105434, -34.9086490979429, 216, 1320, 1680, 2, -1},
{1521, -56.1604564633606, -34.9085764261153, 288, 1320, 1680, 2, -1},
{1522, -56.1609884578852, -34.9086646458006, 288, 1320, 1680, 2, -1},
{1523, -56.1590594581468, -34.9088701347254, 288, 1320, 1680, 2, -1},
{1526, -56.1583561255138, -34.9093086343453, 288, 1320, 1680, 2, -1},
{1528, -56.1621727053376, -34.9075135248004, 288, 1320, 1680, 2, -1},
{1529, -56.1571764739117, -34.9091258434092, 288, 1320, 1680, 2, -1},
{1530, -56.158503993314, -34.9076195285349, 288, 1320, 1680, 2, -1},
{1531, -56.1589438468064, -34.907487714588, 288, 1320, 1680, 2, -1},
{1532, -56.1620464482183, -34.9070017119242, 288, 1320, 1680, 2, -1},
{1533, -56.1599910129002, -34.9075570013894, 288, 1320, 1680, 2, -1},
{1534, -56.1574622494251, -34.9062401232984, 216, 1320, 1680, 2, -1},
{1535, -56.1598770243038, -34.9064095125309, 216, 1320, 1680, 2, -1},
{1536, -56.1543789651922, -34.9072871643997, 288, 1320, 1680, 2, -1},
{1537, -56.1610526510836, -34.9064621444156, 216, 1320, 1680, 2, -1},
{1538, -56.1559817413202, -34.9060352679543, 288, 1320, 1680, 2, -1},
{1539, -56.1550100802536, -34.9060587524505, 288, 1320, 1680, 2, -1},
{1541, -56.1543467292083, -34.908541389081, 216, 1320, 1680, 2, -1},
{1542, -56.1559202958089, -34.9063581475245, 288, 1320, 1680, 2, -1},
{1543, -56.1541132838509, -34.9078723075711, 288, 1320, 1680, 2, -1},
{1544, -56.1553630455384, -34.9086114324103, 288, 1320, 1680, 2, -1},
{1545, -56.1581840848761, -34.9052991547972, 288, 1320, 1680, 2, -1},
{1547, -56.1628797601431, -34.9055770626817, 288, 1320, 1680, 2, -1},
{1548, -56.1572625373085, -34.9072924777255, 288, 1320, 1680, 2, -1},
{1549, -56.1583266758298, -34.905925800165, 288, 1320, 1680, 2, -1},
{1550, -56.1552939333348, -34.9051334798597, 216, 1320, 1680, 2, -1},
{1551, -56.1545277067015, -34.9070893011639, 288, 1320, 1680, 2, -1},
{1552, -56.1545497455872, -34.9070930920044, 288, 1320, 1680, 2, -1},
{1558, -56.1562977666434, -34.9050742935887, 288, 1320, 1680, 2, -1},
{1559, -56.1610162555419, -34.9062287870103, 216, 1320, 1680, 2, -1},
{1560, -56.161757164456, -34.9053194682577, 288, 1320, 1680, 2, -1},
{1561, -56.1625833015234, -34.906143800308, 216, 1320, 1680, 2, -1},
{1562, -56.1618726412413, -34.9059892225077, 288, 1320, 1680, 2, -1},
{1564, -56.1580586827386, -34.9073498339858, 216, 1320, 1680, 2, -1},
{1567, -56.1583996819686, -34.9062496121362, 216, 1320, 1680, 2, -1},
{1568, -56.159422816028, -34.9053376644746, 216, 1320, 1680, 2, -1},
{1571, -56.1588782430571, -34.9052889930866, 288, 1320, 1680, 2, -1},
{1573, -56.1612057580739, -34.9054607453846, 288, 1320, 1680, 2, -1},
{1575, -56.1575334363218, -34.9051910723865, 288, 1320, 1680, 2, -1},
{1577, -56.1596964733466, -34.9062975019062, 216, 1320, 1680, 2, -1},
{1578, -56.1596203469809, -34.9059227782757, 216, 1320, 1680, 2, -1},
{7231, -56.154878126585, -34.9085945437869, 288, 1320, 1680, 2, -1},
{7234, -56.1630954276059, -34.9059944376842, 216, 1320, 1680, 2, -1},
{7235, -56.1628610337999, -34.9050968048811, 288, 1320, 1680, 2, -1},
{7348, -56.1548517919281, -34.9085961793404, 288, 1320, 1680, 2, -1},
{7349, -56.1579506589826, -34.9076280946361, 288, 1320, 1680, 2, -1},
{7350, -56.1591505560924, -34.9072558655045, 288, 1320, 1680, 2, -1},
{7351, -56.162350265998, -34.9087612567638, 288, 1320, 1680, 2, -1},
{7353, -56.1568484847017, -34.9072654752813, 288, 1320, 1680, 2, -1},
{7354, -56.1557664256359, -34.9073385973308, 288, 1320, 1680, 2, -1},
{7356, -56.1579299694103, -34.906270990265, 288, 1320, 1680, 2, -1},
{7359, -56.1594375185329, -34.9050706105096, 216, 1320, 1680, 2, -1},
{7839, -56.1569258329348, -34.9061895254347, 216, 1320, 1680, 2, -1},
{7872, -56.1560315662542, -34.9072031089174, 216, 1320, 1680, 2, -1},
{1498, -56.1618240734045, -34.9100313933631, 216, 1320, 1680, 2, -1},
{1499, -56.1604974997028, -34.9104314700176, 216, 1320, 1680, 2, -1},
{1501, -56.1616846875837, -34.911315292343, 216, 1320, 1680, 2, -1},
{1502, -56.1609625570906, -34.9102170091136, 216, 1320, 1680, 2, -1},
{1504, -56.1607843495579, -34.911686524255, 216, 1320, 1680, 2, -1},
{1506, -56.1614576390175, -34.911468270737, 216, 1320, 1680, 2, -1},
{1508, -56.1605208896102, -34.9113192496716, 216, 1320, 1680, 2, -1},
{1509, -56.1617114828291, -34.9109978397881, 288, 1320, 1680, 2, -1},
{1513, -56.1595329663062, -34.9101989299102, 288, 1320, 1680, 2, -1},
{1515, -56.1577197489612, -34.9100891338149, 288, 1320, 1680, 2, -1},
{1524, -56.1593106336188, -34.910707701792, 216, 1320, 1680, 2, -1},
{1525, -56.1592563320939, -34.9111994677542, 288, 1320, 1680, 2, -1},
{7232, -56.1576866761556, -34.9101048528533, 216, 1320, 1680, 2, -1},
{7236, -56.1593746645453, -34.9096711404284, 288, 1320, 1680, 2, -1},
{9398, -56.15925593294, -34.9111846458599, 216, 1320, 1680, 2, -1},
{1197, -56.1617809767259, -34.9035667847392, 216, 1320, 1680, 2, -1},
{1555, -56.1589547262844, -34.9042028534073, 288, 1320, 1680, 2, -1},
{1556, -56.1602126548285, -34.9030680046669, 288, 1320, 1680, 2, -1},
{1557, -56.1590190826076, -34.9033347915525, 288, 1320, 1680, 2, -1},
{1563, -56.1621715515301, -34.9041083222098, 216, 1320, 1680, 2, -1},
{1566, -56.1605161216024, -34.9043124366113, 288, 1320, 1680, 2, -1},
{1569, -56.1610562841773, -34.9043526448271, 216, 1320, 1680, 2, -1},
{1570, -56.1615436706651, -34.9043913757943, 288, 1320, 1680, 2, -1},
{1576, -56.1580493533153, -34.9031907422402, 288, 1320, 1680, 2, -1},
{7357, -56.1610728851396, -34.9029505120612, 288, 1320, 1680, 2, -1},
{7358, -56.1595557776076, -34.9042468452083, 288, 1320, 1680, 2, -1},
{1540, -56.156990957821, -34.9073664954899, 216, 1320, 1680, 2, -1},
{7233, -56.1573312860243, -34.9079864705559, 216, 1320, 1680, 2, -1},
{7352, -56.1603943090884, -34.9113485753074, 216, 1320, 1680, 2, -1}
};

#endif
