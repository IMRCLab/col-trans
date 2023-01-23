import numpy as np
import yaml

def skew(w):
    w = np.asarray(w).reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

def compute_Pinv(p1a, p2a):
    P = np.zeros((6, 6))
    P[0:3,0:3] = np.eye(3)
    P[0:3,3:6] = np.eye(3)
    P[3:6,0:3] = skew(p1a)
    P[3:6,3:6] = skew(p2a)
    P_inv = np.linalg.pinv(P)
    return P_inv

def Pinv_to_dict(Pinv, idx, id1, id2):
    result = {
        'Pinv{}id1'.format(idx): id1,
        'Pinv{}id2'.format(idx): id2}

    for r in range(6):
        for c in range(6):
            if np.abs(Pinv[r,c]) > 1e-6:
                result['Pinv{}{}{}'.format(idx, r, c)] = float(Pinv[r,c])
    return result

def main():

    with open("gen_Pinv.yaml", 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    s = cfg['triangle']
    result = dict()

    result['massP'] = s['mass']
    for k, a in enumerate(s['attachments']):
        result['ap{}id'.format(k)] = a['id']
        result['ap{}x'.format(k)] = a['point'][0]
        result['ap{}y'.format(k)] = a['point'][1]
        result['ap{}z'.format(k)] = a['point'][2]
        result['ap{}l'.format(k)] = a['length']


    if len(s['attachments']) == 2:
        p1a = np.asarray(s['attachments'][0]['point'])
        id1 = s['attachments'][0]['id']
        p2a = np.asarray(s['attachments'][1]['point'])
        id2 = s['attachments'][1]['id']
        Pinv = compute_Pinv(p1a, p2a)
        result |= Pinv_to_dict(Pinv, 0, id1, id2)

    if len(s['attachments']) == 3:
        p1a = np.asarray(s['attachments'][0]['point'])
        id1 = s['attachments'][0]['id']
        p2a = np.asarray(s['attachments'][1]['point'])
        id2 = s['attachments'][1]['id']
        p3a = np.asarray(s['attachments'][2]['point'])
        id3 = s['attachments'][2]['id']

        Pinv = compute_Pinv(p1a, p2a)
        result |= Pinv_to_dict(Pinv, 0, id1, id2)

        Pinv = compute_Pinv(p1a, p3a)
        result |= Pinv_to_dict(Pinv, 1, id1, id3)

        Pinv = compute_Pinv(p2a, p3a)
        result |= Pinv_to_dict(Pinv, 2, id2, id3)

    print(yaml.dump(result))


if __name__ == '__main__':
    main()