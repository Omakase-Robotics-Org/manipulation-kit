from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_ros_command_response_200 import ChassisRosCommandResponse200
from ...models.error_envelope import ErrorEnvelope
from ...models.vendor_ros_command_type_0 import VendorRosCommandType0
from ...models.vendor_ros_command_type_1 import VendorRosCommandType1
from ...models.vendor_ros_command_type_2 import VendorRosCommandType2
from ...models.vendor_ros_command_type_3 import VendorRosCommandType3
from ...models.vendor_ros_command_type_4 import VendorRosCommandType4
from ...models.vendor_ros_command_type_5 import VendorRosCommandType5
from ...models.vendor_ros_command_type_6 import VendorRosCommandType6
from ...types import Response


def _get_kwargs(
    *,
    body: VendorRosCommandType0
    | VendorRosCommandType1
    | VendorRosCommandType2
    | VendorRosCommandType3
    | VendorRosCommandType4
    | VendorRosCommandType5
    | VendorRosCommandType6,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/ros/command",
    }

    if (
        isinstance(body, VendorRosCommandType0)
        or isinstance(body, VendorRosCommandType1)
        or isinstance(body, VendorRosCommandType2)
        or isinstance(body, VendorRosCommandType3)
        or isinstance(body, VendorRosCommandType4)
        or isinstance(body, VendorRosCommandType5)
    ):
        _kwargs["json"] = body.to_dict()
    else:
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisRosCommandResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisRosCommandResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisRosCommandResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: VendorRosCommandType0
    | VendorRosCommandType1
    | VendorRosCommandType2
    | VendorRosCommandType3
    | VendorRosCommandType4
    | VendorRosCommandType5
    | VendorRosCommandType6,
) -> Response[ChassisRosCommandResponse200 | ErrorEnvelope]:
    """Vendor ROS command

     Fixed typed manufacturer commands only. Requires explicit opt-in, fresh monitor/chassis interlocks
    and chassis soft-kill clearance for motion. AutoCharge end can move and is distinct from HTTP
    navigation stop. One send attempt; timeout returns outcome_unknown without retry. Acknowledgment
    never proves physical completion. Archived vendor contracts are not verified against the installed
    binary. For command=pose_reset read the operation-specific request variant and report constraints:
    fixed /reset_pose publish, never a service ACK. Only transport_sent or outcome_unknown with required
    vendor_success=null is valid. A complete authentic daemon error envelope for pose_reset means
    prepublish rejection/setup failure; once publish send is attempted, uncertainty is returned in an
    HTTP200/status ok report. HTTP success is not operation success. Missing, malformed, proxy or
    network responses cannot establish prepublish refusal. Reread fresh state and review before another
    explicit request; never automatically retry. The same rules apply to WebSocket chassis.ros.command.

    Args:
        body (VendorRosCommandType0 | VendorRosCommandType1 | VendorRosCommandType2 |
            VendorRosCommandType3 | VendorRosCommandType4 | VendorRosCommandType5 |
            VendorRosCommandType6): Fixed request catalogue. No service name, topic name or arbitrary
            ROS arguments.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRosCommandResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: VendorRosCommandType0
    | VendorRosCommandType1
    | VendorRosCommandType2
    | VendorRosCommandType3
    | VendorRosCommandType4
    | VendorRosCommandType5
    | VendorRosCommandType6,
) -> ChassisRosCommandResponse200 | ErrorEnvelope | None:
    """Vendor ROS command

     Fixed typed manufacturer commands only. Requires explicit opt-in, fresh monitor/chassis interlocks
    and chassis soft-kill clearance for motion. AutoCharge end can move and is distinct from HTTP
    navigation stop. One send attempt; timeout returns outcome_unknown without retry. Acknowledgment
    never proves physical completion. Archived vendor contracts are not verified against the installed
    binary. For command=pose_reset read the operation-specific request variant and report constraints:
    fixed /reset_pose publish, never a service ACK. Only transport_sent or outcome_unknown with required
    vendor_success=null is valid. A complete authentic daemon error envelope for pose_reset means
    prepublish rejection/setup failure; once publish send is attempted, uncertainty is returned in an
    HTTP200/status ok report. HTTP success is not operation success. Missing, malformed, proxy or
    network responses cannot establish prepublish refusal. Reread fresh state and review before another
    explicit request; never automatically retry. The same rules apply to WebSocket chassis.ros.command.

    Args:
        body (VendorRosCommandType0 | VendorRosCommandType1 | VendorRosCommandType2 |
            VendorRosCommandType3 | VendorRosCommandType4 | VendorRosCommandType5 |
            VendorRosCommandType6): Fixed request catalogue. No service name, topic name or arbitrary
            ROS arguments.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRosCommandResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: VendorRosCommandType0
    | VendorRosCommandType1
    | VendorRosCommandType2
    | VendorRosCommandType3
    | VendorRosCommandType4
    | VendorRosCommandType5
    | VendorRosCommandType6,
) -> Response[ChassisRosCommandResponse200 | ErrorEnvelope]:
    """Vendor ROS command

     Fixed typed manufacturer commands only. Requires explicit opt-in, fresh monitor/chassis interlocks
    and chassis soft-kill clearance for motion. AutoCharge end can move and is distinct from HTTP
    navigation stop. One send attempt; timeout returns outcome_unknown without retry. Acknowledgment
    never proves physical completion. Archived vendor contracts are not verified against the installed
    binary. For command=pose_reset read the operation-specific request variant and report constraints:
    fixed /reset_pose publish, never a service ACK. Only transport_sent or outcome_unknown with required
    vendor_success=null is valid. A complete authentic daemon error envelope for pose_reset means
    prepublish rejection/setup failure; once publish send is attempted, uncertainty is returned in an
    HTTP200/status ok report. HTTP success is not operation success. Missing, malformed, proxy or
    network responses cannot establish prepublish refusal. Reread fresh state and review before another
    explicit request; never automatically retry. The same rules apply to WebSocket chassis.ros.command.

    Args:
        body (VendorRosCommandType0 | VendorRosCommandType1 | VendorRosCommandType2 |
            VendorRosCommandType3 | VendorRosCommandType4 | VendorRosCommandType5 |
            VendorRosCommandType6): Fixed request catalogue. No service name, topic name or arbitrary
            ROS arguments.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRosCommandResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: VendorRosCommandType0
    | VendorRosCommandType1
    | VendorRosCommandType2
    | VendorRosCommandType3
    | VendorRosCommandType4
    | VendorRosCommandType5
    | VendorRosCommandType6,
) -> ChassisRosCommandResponse200 | ErrorEnvelope | None:
    """Vendor ROS command

     Fixed typed manufacturer commands only. Requires explicit opt-in, fresh monitor/chassis interlocks
    and chassis soft-kill clearance for motion. AutoCharge end can move and is distinct from HTTP
    navigation stop. One send attempt; timeout returns outcome_unknown without retry. Acknowledgment
    never proves physical completion. Archived vendor contracts are not verified against the installed
    binary. For command=pose_reset read the operation-specific request variant and report constraints:
    fixed /reset_pose publish, never a service ACK. Only transport_sent or outcome_unknown with required
    vendor_success=null is valid. A complete authentic daemon error envelope for pose_reset means
    prepublish rejection/setup failure; once publish send is attempted, uncertainty is returned in an
    HTTP200/status ok report. HTTP success is not operation success. Missing, malformed, proxy or
    network responses cannot establish prepublish refusal. Reread fresh state and review before another
    explicit request; never automatically retry. The same rules apply to WebSocket chassis.ros.command.

    Args:
        body (VendorRosCommandType0 | VendorRosCommandType1 | VendorRosCommandType2 |
            VendorRosCommandType3 | VendorRosCommandType4 | VendorRosCommandType5 |
            VendorRosCommandType6): Fixed request catalogue. No service name, topic name or arbitrary
            ROS arguments.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRosCommandResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
