python .\gtfs\gtfs2pt.py ^
-n plzen_trd_paper.net.xml ^
--gtfs pmdp_gtfs_202210.zip ^
--date 20221109 ^
--modes trolleybus ^
--vtype-output      plzen_gtfs_pt_vtypes.add.xml ^
--warning-output    plzen_gtfs_pt.warn.log ^
--additional-output plzen_gtfs_pt_stops_routes.add.xml ^
--route-output      plzen_gtfs_pt_vehicles.rou.xml ^
--verbose
