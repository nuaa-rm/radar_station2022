export default {
  namespace: 'configProvider',
  state: {calibrator: {rate: 3, clipRectRate: 0.5, cameras:{}}},
  reducers: {
    'init' (state, {payload: {calibration, cameras}}) {
      return {...state, calibrator: {...calibration, cameras}}
    },
  },
};
