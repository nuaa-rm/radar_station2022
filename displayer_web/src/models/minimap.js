export default {
  namespace: 'minimap',
  state: {test1: {color: 'red', text: 'test', data: [[0.2, 0.5], [0.6, 0.7], [0.2, 0.7]], shapeType: 'polygon'}, test2: {color: 'green', data: [[0.3, 0.5]], text: '1', shapeType: 'point'}},
  reducers: {
    'refresh' (state, { payload }) {
      let re = {...state}
      for (let i = 0; i < payload.length; i++) {
        re[payload[i].id] = payload[i]
      }
      return re
    },
  },
};
