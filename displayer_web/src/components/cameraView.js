import React, {Component} from 'react';
import { connect } from 'umi';

@connect(({ configProvider }) => ({
  configProvider,
}))
class CameraView extends Component {
  state = {camera: ''}

  render() {
    let uri;
    if (this.state.camera === '') {
      uri = require('../assets/test.png');
    }
    return (
      <div style={{height: '100%', width: '100%'}}>
        <img src={uri} alt="camera" style={{maxHeight: '100%', maxWidth: '100%'}}/>
      </div>
    );
  }
}

export default CameraView;
