export default {
  namespace: 'backendModel',
  state: {ok: 'loading'},
  reducers: {
    'rosConnectChange'(state, { payload: connected }) {
      return {...state, ok: connected};
    },
  },
};
