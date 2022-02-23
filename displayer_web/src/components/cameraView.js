import React, {Component} from 'react';
import { connect } from 'umi';
import {
  ModalForm,
  ProFormSelect,
} from '@ant-design/pro-form';

@connect(({ configProvider }) => ({
  configProvider,
}))
class CameraView extends Component {
  state = {camera: '', modalShow: false}

  onModalShowChange(e) {
    console.log(e)
    this.setState({modalShow: e})
  }

  onSubmit(e) {
    this.setState({
      camera: e.camera,
      modalShow: false,
    })
  }

  render() {
    let uri;
    if (this.state.camera === '') {
      uri = require('../assets/noCamera.png');
    } else {
      uri = '/api/camera?cam=' + encodeURIComponent(this.state.camera)
    }
    let cameras = {}
    const cameraList = Object.keys(this.props.configProvider.calibrator.cameras)
    for (let i = 0; i < cameraList.length; i++) {
      cameras[cameraList[i]] = cameraList[i]
    }
    return (
      <div style={{height: '100%', width: '100%'}}>
        <img
          src={uri} alt="camera" style={{maxHeight: '100%', maxWidth: '100%', cursor: 'pointer'}}
          onClick={()=>{this.onModalShowChange(true)}}
        />
        <ModalForm
          title="Select Camera"
          onFinish={async values=>{this.onSubmit(values)}}
          visible={this.state.modalShow}
          onVisibleChange={e=>this.onModalShowChange(e)}
        >
          <ProFormSelect
            name="camera"
            label="Camera"
            placeholder="Please Select Camera"
            valueEnum={cameras}
          />
        </ModalForm>
      </div>
    );
  }
}

export default CameraView;
