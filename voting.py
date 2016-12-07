#coding:utf-8
# 関数voting のテスト

import random
epsiron = 20

episode_set =[
[0,0,0,0,"f",0],
#[0,0,0,0,"f",0],
#[0,0,0,0,"r",1],
#[0,0,0,0,"r",-1],
]
particle = range(100)
for i in (particle):
    particle[i] = [(i+1)%5,0.001]

T = 4

print particle

def voting(particle):
    vote = range(100) #各パーティクルが自分の所属しているエピソードに対して持つ評価
    for i in (vote):
        vote[i] = 0.0

    if T == 1: #まだどんなエピソードも経験していないのでランダムに行動させる
        return random.choice("frl")
    else:#各パーティクルが投票で決める
        for i in range(100):
            distance = 0 #パーティクルがいるエピソードとその直後の非ゼロ報酬が得られたエピソードとの距離
            non_zero_reword = 0.0
            for l in range(len(episode_set) - particle[i][0] - 1):
                distance += 1 
                if episode_set[ particle[i][0] + distance ][5] != 0:
                    non_zero_reword = float(episode_set[particle[i][0] + distance][5])
                    break
            print "particle:",i," position:",particle[i][0]," distance:",distance," non_zero_reword:",non_zero_reword
            if non_zero_reword != 0:
                vote[i] = non_zero_reword / distance
            else:
                vote[i] = 0.0
            print "vote:",vote[i]

    #voteに基づく行動決定。voteの合計がゼロやマイナスになる可能性がある点に注意
    got = {"f":0.0,"r":0.0,"l":0.0}
    for i in range(100):
        if int(vote[i]) != 0:
            print episode_set[particle[i][0]] [4]
            got [episode_set[particle[i][0]] [4]] += vote[i]
    print got
    # グリーディならgotの中で最大の数字を持つもののキーをひとつ返す
    # 参考: http://cointoss.hatenablog.com/entry/2013/10/16/123129
    if (random.randint(1,100) > epsiron):
        return max(got.items(),key=lambda x:x[1])[0]
    else:
        print "random_choice"
        return random.choice("frl")

action = voting(particle)
print action 
