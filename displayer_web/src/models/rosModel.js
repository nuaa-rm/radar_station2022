const noConnectionImage = require('../assets/test.png')

export default {
  namespace: 'rosModel',
  state: {ok: 'loading', cameraOne: noConnectionImage, cameraTwo: noConnectionImage},
  reducers: {
    'rosConnectChange'(state, { payload: connected }) {
      return {...state, ok: connected};
    },
    'rosCameraChange'(state, { payload: { camera, image } }) {
      if (camera === 'cameraOne') {
        return {...state, cameraOne: image};
      } else {
        return {...state, cameraTwo: image};
      }
    }
  },
};
