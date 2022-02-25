import React, { Component, createRef } from 'react';
import { connect } from 'umi';
import {
  ModalForm,
  ProFormSelect,
} from '@ant-design/pro-form';

@connect(({ configProvider }) => ({
  configProvider,
}))
class CameraView extends Component {
  state = {camera: '', modalShow: false, aspectRatio: 4. / 3.};
  container = createRef();

  onModalShowChange(e) {
    this.setState({modalShow: e})
  }

  onSubmit(e) {
    let aspectRatio = 4. / 3.;
    if (e.camera !== '') {
      aspectRatio = this.props.configProvider.calibrator.cameras[e.camera].aspectRatio;
    }
    this.setState({
      camera: e.camera,
      modalShow: false,
      aspectRatio
    })
    this.resize(aspectRatio)
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

    const aspectRatio = this.state.aspectRatio;
    let width = this.props.width
    let height = this.props.height
    let targetWidth = height * aspectRatio;
    if (width < targetWidth) {
      targetWidth = width;
      height = targetWidth / aspectRatio;
    }
    return (
      <div style={{height: '100%', width: '100%'}} ref={this.container}>
        <img
          src={uri} alt="camera" style={{height: height, width: targetWidth, cursor: 'pointer'}}
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
