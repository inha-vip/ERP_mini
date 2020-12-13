#-*- coding:utf-8 -*-
import glob
import csv
import math
import matplotlib.pyplot as plt

class Global_planner:

    def __init__(self, start_x, start_y, start_yaw):
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.goal_x, self.goal_y = self.goal_pt()
        self.files = glob.glob("*.csv")
        self.lane_list = []
        self.locked_lane = ''



    def goal_pt(self):
        goal_x = float(input('press x (Recommended to enter negative values):'))
        goal_y = float(input('press y (Recommended to enter negative values):'))

        return goal_x, goal_y

    def lane_info(self):
        fig = plt.figure()
        plt.grid(True)
        plt.tight_layout()
        for file in self.files:
            tmp_name = file[0:4]
            # print(type(tmp_name))

            globals()['Lane{}'.format(tmp_name)]  = {'id': tmp_name, 'x':[], 'y':[], 
            'yaw':[], 'k':[],'s':[], 'g_cost': 0.0 ,'pre_lane': [], 'next_lane': []}

            self.lane_list.append(tmp_name)

            csv_file = '/home/vip/Desktop/ERP_mini/map/'+ file
            with open(csv_file, mode='r') as map_file:

            # csv_file = 'C:/Users/junma/OneDrive/바탕 화면/ERP_mini/ERP_mini/map/'+ file
            # with open(csv_file, mode='r') as map_file:
                csv_reader = csv.reader(map_file)
                for line in csv_reader:
                    globals()['Lane{}'.format(tmp_name)]['x'].append(float(line[0]))
                    globals()['Lane{}'.format(tmp_name)]['y'].append(float(line[1]))
                    if math.degrees(float(line[2])) < 0:
                        deg_yaw = math.degrees(float(line[2])) + 360
                    else:
                        deg_yaw = math.degrees(float(line[2]))
                        
                    if deg_yaw > 90 and deg_yaw <=360:
                       deg_yaw -= 90
                    else:
                       deg_yaw+= 270 
                    globals()['Lane{}'.format(tmp_name)]['yaw'].append(deg_yaw)
                    globals()['Lane{}'.format(tmp_name)]['k'].append(float(line[3]))
                    globals()['Lane{}'.format(tmp_name)]['s'].append(float(line[4]))
                    globals()['Lane{}'.format(tmp_name)]['g_cost'] = globals()['Lane{}'.format(tmp_name)]['s'][-1]

                plt.scatter(globals()['Lane{}'.format(tmp_name)]['x'], globals()['Lane{}'.format(tmp_name)]['y'], marker='.')
            map_file.close()

       
        
            
        Lane0001['pre_lane'].append('1000')
        Lane0001['next_lane'].append('0102')
        Lane0001['next_lane'].append('0111')
         
         
        Lane0010['pre_lane'].append('0100')
        Lane0010['next_lane'].append('1020')
        Lane0010['next_lane'].append('1011')
         
         
        Lane1424['pre_lane'].append('0414')
        Lane1424['pre_lane'].append('1314')
        Lane1424['next_lane'].append('2423')
         
        Lane2324['pre_lane'].append('1323')
        Lane2324['pre_lane'].append('2223')
        Lane2324['next_lane'].append('2414')
         
         
        Lane0102['pre_lane'].append('0001') 
        Lane0102['pre_lane'].append('1101')
        Lane0102['next_lane'].append('0203')
        Lane0102['next_lane'].append('0212')
         
        Lane0203['pre_lane'].append('1202') 
        Lane0203['pre_lane'].append('0102')
        Lane0203['next_lane'].append('0304')
        Lane0203['next_lane'].append('0313')
         
        Lane0304['pre_lane'].append('0203') 
        Lane0304['pre_lane'].append('1303')
        Lane0304['next_lane'].append('0414')
         
        Lane1011['pre_lane'].append('0010') 
        Lane1011['pre_lane'].append('2010') 
        Lane1011['next_lane'].append('1101')
        Lane1011['next_lane'].append('1121')
        Lane1011['next_lane'].append('1112')
         
        Lane1112['pre_lane'].append('0111')
        Lane1112['pre_lane'].append('1011') 
        Lane1112['pre_lane'].append('2111') 
        Lane1112['next_lane'].append('1202')
        Lane1112['next_lane'].append('1213')
        Lane1112['next_lane'].append('1222')
         
        Lane1213['pre_lane'].append('0212')
        Lane1213['pre_lane'].append('1112') 
        Lane1213['pre_lane'].append('2212') 
        Lane1213['next_lane'].append('1303')
        Lane1213['next_lane'].append('1314')
        Lane1213['next_lane'].append('1323')
         
        Lane1314['pre_lane'].append('0313')
        Lane1314['pre_lane'].append('1213') 
        Lane1314['pre_lane'].append('2313') 
        Lane1314['next_lane'].append('1404')
        Lane1314['next_lane'].append('1424')
         
        Lane2021['pre_lane'].append('1020')
        Lane2021['next_lane'].append('2111')
        Lane2021['next_lane'].append('2122')
         
        Lane2122['pre_lane'].append('1121')
        Lane2122['pre_lane'].append('2021')
        Lane2122['next_lane'].append('2212')
        Lane2122['next_lane'].append('2223')
         
        Lane2223['pre_lane'].append('1222')
        Lane2223['pre_lane'].append('2122')
        Lane2223['next_lane'].append('2313')
        Lane2223['next_lane'].append('2324')
         
        Lane1020['pre_lane'].append('1110')
        Lane1020['pre_lane'].append('0010')
        Lane1020['next_lane'].append('2021')
         
        Lane0111['pre_lane'].append('0001')
        Lane0111['pre_lane'].append('0201')
        Lane0111['next_lane'].append('1112')
        Lane0111['next_lane'].append('1121')
        Lane0111['next_lane'].append('1110')
         
        Lane0212['pre_lane'].append('0102')
        Lane0212['pre_lane'].append('0302')
        Lane0212['next_lane'].append('1213')
        Lane0212['next_lane'].append('1222')
        Lane0212['next_lane'].append('1211')
         
        Lane0313['pre_lane'].append('0203')
        Lane0313['pre_lane'].append('0403')
        Lane0313['next_lane'].append('1314')
        Lane0313['next_lane'].append('1323')
        Lane0313['next_lane'].append('1312')
         
        Lane0414['pre_lane'].append('0304')
        Lane0414['next_lane'].append('1413')
        Lane0313['next_lane'].append('1424')
         
        Lane1121['pre_lane'].append('0111')
        Lane1121['pre_lane'].append('1011')
        Lane1121['pre_lane'].append('1211')
        Lane1121['next_lane'].append('2122')
        Lane1121['next_lane'].append('2120')
         
        Lane1222['pre_lane'].append('0212')
        Lane1222['pre_lane'].append('1312')
        Lane1222['pre_lane'].append('1112')
        Lane1222['next_lane'].append('2223')
        Lane1222['next_lane'].append('2221')
         
        Lane1323['pre_lane'].append('0313')
        Lane1323['pre_lane'].append('1213')
        Lane1323['pre_lane'].append('1413')
        Lane1323['next_lane'].append('2324')
        Lane1323['next_lane'].append('2322')
         
         
        #BLUELANE
        Lane1000['pre_lane'].append('1110')
        Lane1000['pre_lane'].append('2010')
        Lane1000['next_lane'].append('0001')
         
        Lane2010['pre_lane'].append('2120')
        Lane2010['next_lane'].append('1011')
        Lane2010['next_lane'].append('1000')
         
        Lane2120['pre_lane'].append('2221')
        Lane2120['pre_lane'].append('1121')
        Lane2120['next_lane'].append('2010')
         
        Lane1110['pre_lane'].append('0111')
        Lane1110['pre_lane'].append('1211')
        Lane1110['pre_lane'].append('2111')
        Lane1110['next_lane'].append('1020')
        Lane1110['next_lane'].append('1000')
         
        Lane0100['pre_lane'].append('1101')
        Lane0100['pre_lane'].append('0201')
        Lane0100['next_lane'].append('0010')
         
        Lane1101['pre_lane'].append('1211')
        Lane1101['pre_lane'].append('2111')
        Lane1101['pre_lane'].append('1011')
        Lane1101['next_lane'].append('0102')
        Lane1101['next_lane'].append('0100')
         
         
        Lane2111['pre_lane'].append('2221')
        Lane2111['pre_lane'].append('2021')
        Lane2111['next_lane'].append('1112')
        Lane2111['next_lane'].append('1101')
        Lane2111['next_lane'].append('1110')
         
         
        Lane2221['pre_lane'].append('2322')
        Lane2221['pre_lane'].append('1222')
        Lane2221['next_lane'].append('2120')
        Lane2221['next_lane'].append('2111')
         
        Lane1211['pre_lane'].append('0212')
        Lane1211['pre_lane'].append('1312')
        Lane1211['pre_lane'].append('2212')
        Lane1211['next_lane'].append('1101')
        Lane1211['next_lane'].append('1121')
        Lane1211['next_lane'].append('1110')
         
        Lane0201['pre_lane'].append('1202')
        Lane0201['pre_lane'].append('0302')
        Lane0201['next_lane'].append('0100')
        Lane0201['next_lane'].append('0111')
         
        Lane1202['pre_lane'].append('1312')
        Lane1202['pre_lane'].append('2212')
        Lane1202['pre_lane'].append('1112')
        Lane1202['next_lane'].append('0203')
        Lane1202['next_lane'].append('0201')
         
        Lane2212['pre_lane'].append('2322')
        Lane2212['pre_lane'].append('2122')
        Lane2212['next_lane'].append('1213')
        Lane2212['next_lane'].append('1202')
        Lane2212['next_lane'].append('1211')
         
        Lane2322['pre_lane'].append('2423')
        Lane2322['pre_lane'].append('1323')
        Lane2322['next_lane'].append('2221')
        Lane2322['next_lane'].append('2212')
         
        Lane1312['pre_lane'].append('2313')
        Lane1312['pre_lane'].append('1413')
        Lane1312['pre_lane'].append('0313')
        Lane1312['next_lane'].append('1222')
        Lane1312['next_lane'].append('1211')
        Lane1312['next_lane'].append('1202')
         
        Lane0302['pre_lane'].append('1303')
        Lane0302['pre_lane'].append('0403')
        Lane0302['next_lane'].append('0212')
        Lane0302['next_lane'].append('0201')
         
        Lane0403['pre_lane'].append('1404')
        Lane0403['next_lane'].append('0313')
        Lane0403['next_lane'].append('0302')
         
        Lane2423['pre_lane'].append('1424')
        Lane2423['next_lane'].append('2313')
        Lane2423['next_lane'].append('2322')
         
        Lane2414['pre_lane'].append('2324')
        Lane2414['next_lane'].append('1404')
        Lane2414['next_lane'].append('1413')
         
        Lane1404['pre_lane'].append('2414')
        Lane1404['pre_lane'].append('1314')
        Lane1404['next_lane'].append('0403')
         
        Lane1413['pre_lane'].append('2414')
        Lane1413['pre_lane'].append('0414')
        Lane1413['next_lane'].append('1323')
        Lane1413['next_lane'].append('1312')
        Lane1413['next_lane'].append('1303')
         
        Lane1303['pre_lane'].append('1413')
        Lane1303['pre_lane'].append('2313')
        Lane1303['pre_lane'].append('1213')
        Lane1303['next_lane'].append('0304')
        Lane1303['next_lane'].append('0302')
         
        Lane2313['pre_lane'].append('2423')
        Lane2313['pre_lane'].append('2223')
        Lane2313['next_lane'].append('1314')
        Lane2313['next_lane'].append('1303')
        Lane2313['next_lane'].append('1312')


    def select_lanelet(self, x, y, yaw, goal):
        min_val = 99999
        for lane_num in self.lane_list:
            for i in range(len(globals()['Lane{}'.format(lane_num)]['x'])):
                d = math.hypot(x-globals()['Lane{}'.format(lane_num)]['x'][i], 
                    y-globals()['Lane{}'.format(lane_num)]['y'][i])

                if d <= min_val:
                    min_val = d
                    if goal:
                        cur_lane = globals()['Lane{}'.format(lane_num)] 
                        self.locked_lane  = lane_num[2:4] + lane_num[0:2]
                    else:
                        th =  globals()['Lane{}'.format(lane_num)]['yaw'][i]
                        if abs(th - yaw) < 90:
                            cur_lane = globals()['Lane{}'.format(lane_num)]
                            self.locked_lane  = lane_num[2:4] + lane_num[0:2]
                        else:
                            cur_lane = globals()['Lane{}'.format(lane_num[2:4] + lane_num[0:2])]
                            self.locked_lane  = lane_num

                    
                    min_idx = i
        #print('min_idx:', min_idx) 
        return cur_lane, min_idx

    @staticmethod
    def calc_heuristic(x1, y1, x2, y2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(x1 - x2, y1 - y2)
        return d

    #node end of lane
    def next_Node(self, lane):
        return {'node_id': lane['id'][2:4], 'lane_name':lane['id'], 'x': lane['x'][-1], 'y': lane['y'][-1], 
        'parent_node': lane['id'][0:2], 'g_cost': float(lane['s'][-1]) }

    #node fisrt point of lane ### only for finding goal node
    def pre_Node(self, lane):
        return {'node_id': lane['id'][0:2], 'x': lane['x'][0], 'y': lane['y'][0] }


    def planning(self, sx, sy, syaw, gx, gy):


        start_lane , s_min_idx = self.select_lanelet(sx, sy, syaw, goal=False)
        s_node = self.next_Node(start_lane)
        head_lane = [start_lane['x'][s_min_idx:] , start_lane['y'][s_min_idx:], start_lane['yaw'][s_min_idx:], 
        start_lane['k'][s_min_idx:], start_lane['s'][s_min_idx:] , start_lane['id']]

        
        
        goal_lane , g_min_idx = self.select_lanelet(gx, gy, syaw, goal=True)
        g_node= self.pre_Node(goal_lane)
        tail_lane = [goal_lane['x'][:g_min_idx] , goal_lane['y'][:g_min_idx] , goal_lane['yaw'][:g_min_idx], 
        goal_lane['k'][:g_min_idx], goal_lane['s'][:g_min_idx] , goal_lane['id']]

        print('start_lane:', start_lane['id'])
        print('goal_lane:', goal_lane['id'])

        open_set, closed_set = dict(), dict()

        closed_set[s_node['node_id']] = s_node


        # print('s_node:',s_node)
        # print(self.lane_list)

        for lane in self.lane_list:
            # print('lane:', lane[0:2])
            if s_node['node_id'] == lane[0:2]:
                open_set[lane[2:4]] = self.next_Node(globals()['Lane{}'.format(lane)])
        
        del open_set[self.pre_Node(start_lane)['node_id']]
       
        # print('open_set:',open_set)
        print('open_set key:', open_set.keys())
        print('closed_set key:', closed_set.keys())
        print('-'*60)

        while 1:
            #break if openset is empty -> no way to go
            if len(open_set) == 0:
                print("Open set is empty")
                break
            
            ## if goal node is in closed_set, stop calculating
            if g_node['node_id'] in closed_set:
                print('finish GPP')
                break

            
            
            cur_node_id = min(
                open_set.keys(),
                key=(lambda k: open_set[k]['g_cost'] 
                + self.calc_heuristic(open_set[k]['x'] , open_set[k]['y'] , g_node['x'] , g_node['y']) ) )

            selected_lane = open_set[cur_node_id]['lane_name']
            
            
            print('x'*10)
            print('selected_lanelet:',selected_lane)
            print('cur id:', cur_node_id)
            print('x'*10)
            
            closed_set[cur_node_id] = self.next_Node(globals()['Lane{}'.format(open_set[cur_node_id]['lane_name'])])
    
            print(self.next_Node(Lane1011)['g_cost'])
            del open_set[cur_node_id]
            print(self.next_Node(Lane1011)['g_cost'])
            
            print('#####################case sort#######################')
            for lane in globals()['Lane{}'.format(selected_lane)]['next_lane']:
                new_lane = globals()['Lane{}'.format(lane)]
                if lane == self.locked_lane:
                    print('case4')
                    pass
                
                elif self.next_Node(new_lane)['node_id'] in closed_set:
                    print('case1: already in closed_set')
                    pass

                elif self.next_Node(new_lane)['node_id'] in open_set:
                    #compare with previous value
                    print('case2: compare with previous value')
                    tmp_id = self.next_Node(new_lane)['node_id']
                    old_val = open_set[tmp_id]['g_cost']
                    new_val = self.next_Node(globals()['Lane{}'.format(selected_lane)])['g_cost'] + self.next_Node(new_lane)['g_cost']
                    
                    print('old_val',  old_val, 'new_val:', new_val)
                    if new_val <= old_val:

                        del open_set[tmp_id]
                        open_set[tmp_id] = self.next_Node(new_lane)
                        open_set[tmp_id]['g_cost'] = new_val 
                    else:
                        pass
                else:
                    print('case3: update previous values')
                    open_set[lane[2:4]] = self.next_Node(new_lane)
                    print('open set key:', open_set.keys())
                    print('lane:', lane ,'node:', lane[2:4],'before update:', open_set[lane[2:4]]['g_cost'])
                    print('new lane:', lane, 'selected_lane:', selected_lane)
                    print(selected_lane,'<-selected_lane g_cost:', self.next_Node(globals()['Lane{}'.format(selected_lane)])['g_cost'])
                    open_set[lane[2:4]]['g_cost']  += globals()['Lane{}'.format(selected_lane)]['g_cost']
                    
                    globals()['Lane{}'.format(lane)]['g_cost'] = open_set[lane[2:4]]['g_cost'] 
                    
                    print('lane',  lane ,  'g cost:',  globals()['Lane{}'.format(lane)]['g_cost'])
                    print(lane[2:4], 'after update:', 'new g_cost of', lane, ':', globals()['Lane{}'.format(lane)]['g_cost'])
                    
                
                    


                

                # print('new_g_cost', lane, ':', open_set[lane[2:4]]['g_cost'])


            print('-'*30)
            #print('open_set check:',open_set)
            print('open_set key check:',open_set.keys())
            print('closed_set key check:', closed_set.keys())
            print('x-'*20)

        f_x, f_y, f_yaw, f_k, f_s = self.calc_final_path(cur_node_id, closed_set, head_lane, tail_lane)
           

        plt.scatter(f_x, f_y, marker='x') 
        
            # way point 번호 확인용 
        
        for i in range(int(len(f_x)/10)):
            plt.text(f_x[i*10]+0.05, f_y[i*10]+0.05, round(f_yaw[i*10],  2))
            
        #for i in range(int(len(f_x)/10)):
        #    plt.text(f_x[i*10]+0.05, f_y[i*10]+0.05, i)
        # plt.text(f_x[0]+0.05, f_y[0]+0.05,  'start' ,  fontsize = 10)
        # plt.text(f_x[-1]+0.05, f_y[-1]+0.05,  'goal' ,  fontsize = 10)
        # plt.text(f_x[50]-0.05, f_y[50]-0.05, 'first yaw: {}'.format(f_yaw[0])   ,  fontsize = 10)
        # plt.text(f_x[150]-0.05, f_y[150]-0.05, '150 yaw{}'.format(f_yaw[100])   ,  fontsize = 10)
        # plt.text(f_x[0]-1, f_y[0]-1 , 'start_yaw: {}'.format(self.start_yaw)   ,  fontsize = 10)
        # plt.show()
        

        return f_x, f_y, f_yaw, f_k, f_s


            

    def calc_final_path(self, cur_node_id, closed_set, head, tail):
        tmp_node = cur_node_id
        final_lane_list = []
        f_x, f_y, f_yaw, f_k, f_s  = [],[],[],[],[]
        while 1:
            if tmp_node not in closed_set:

                break
            final_lane_list.append(closed_set[tmp_node]['parent_node'] + tmp_node)
            tmp_node = closed_set[tmp_node]['parent_node']
        #final_lane_list.reverse()
        #print('lane list:',"->".join(final_lane_list))
        #final_lane_list.reverse()
        #final_lane_list.pop()
        final_lane_list.reverse()
        # final_lane_list.pop()

        # f_x = f_x + tail[0]
        # f_y = f_y 
        # f_yaw = f
        #f_x =   f_x + head[0]
        #f_y =   f_y + head[1]
        #f_yaw =  f_yaw + head[2]
        #f_k =  f_k + head[3]
        #f_s = f_s + head[4]

        for lane in final_lane_list:
            f_x = f_x + (globals()['Lane{}'.format(lane)]['x'])
            f_y = f_y + (globals()['Lane{}'.format(lane)]['y'])
            f_yaw = f_yaw + (globals()['Lane{}'.format(lane)]['yaw'])
            f_k = f_k + (globals()['Lane{}'.format(lane)]['k'])

            if len(f_s) == 0:
                f_s = f_s + (globals()['Lane{}'.format(lane)]['s'])
            else:
                for i_s in range(len((globals()['Lane{}'.format(lane)]['s']))):
                    globals()['Lane{}'.format(lane)]['s'][i_s] += f_s[-1]
                f_s = f_s + globals()['Lane{}'.format(lane)]['s']


            


        f_x =   f_x + tail[0]
        f_y =   f_y + tail[1]
        f_yaw =  f_yaw + tail[2]
        f_k =  f_k + tail[3]
        for ii_s in range(len(tail[4])):
            tail[4][ii_s] += f_s[-1]
        f_s = f_s + tail[4]

        # print('f_s:', f_s)
        

        # print('headfadfaf', tail[5])
        # final_lane_list = head[5].tolist() + final_lane_list + tail[5].tolist()

        #print('lane list:',"->".join(final_lane_list))
        # print('f_x:', f_yaw)
        print('len f_x:',len(f_x))

        return f_x, f_y, f_yaw, f_k, f_s




if __name__ == "__main__":
    gpp = Global_planner(-26,-18, 350)
    gpp.lane_info()

    fx, fy, fyaw, fk, fs = gpp.planning(gpp.start_x, gpp.start_y, gpp.start_yaw, gpp.goal_x, gpp.goal_y)
    

    plt.show()



