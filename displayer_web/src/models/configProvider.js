const copyArr = (arr) => arr.map(item => ([...item]))

export default {
  namespace: 'configProvider',
  state: {calibrator: {rate: 3, clipRectRate: 0.5, cameras:{}}},
  reducers: {
    'init' (state, {payload: {calibration, cameras}}) {
      return {...state, calibrator: {...calibration, cameras}}
    },
    'setPath' (state, {payload: {camera, path}}) {
      let re = {...state}
      re.calibrator.cameras[camera].path = copyArr(path)
      return re
    },
  },
};
