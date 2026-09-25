from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.gripper_target import GripperTarget
from ...models.gripper_target_response_200 import GripperTargetResponse200
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: GripperTarget,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/gripper/{side}/target".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | GripperTargetResponse200 | None:
    if response.status_code == 200:
        response_200 = GripperTargetResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | GripperTargetResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripperTarget,
) -> Response[ErrorEnvelope | GripperTargetResponse200]:
    """Post the newest streamed target for one gripper

     The non-blocking lane, for controllers that command at a fixed rate. It takes the same body as `set`
    and returns as soon as the target is recorded, not when the jaws arrive: a per-side worker drives
    the same force-limited stroke machine toward whatever the latest target is. Rapid updates coalesce
    (newest wins, one continuous motion), and a target that crosses the current jaw position re-aims or
    reverses the stroke in flight rather than queueing behind it. Same force limit, grasp detection,
    hold preload, coil guard and soft-kill gate as `set`. Read `/v1/gripper/{side}/state` for
    `target_closedness`, `tracking` and the last stroke outcome. Use `set` when the caller wants the
    stroke's result.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripperTarget): A continuous gripper target.

            Exactly one of `closedness` and `jaw_rad` is given. `closedness` is the hand-protocol
            convention shared with dx-manipulator's wire units: `0.0` is the commanded
            open ceiling, `1.0` the closed stop, linear in motor radians between them.
            `jaw_rad` addresses the motor directly, in the same radians the stroke
            report's `jaw_rad` uses. The backend resolves either to a motor target and
            refuses values outside the mechanism's range. The flattened
            [`GripRequest`] fields (`grip`, `grip_preload_rad`) say how hard to hold
            whatever the stroke meets on the way.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperTargetResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripperTarget,
) -> ErrorEnvelope | GripperTargetResponse200 | None:
    """Post the newest streamed target for one gripper

     The non-blocking lane, for controllers that command at a fixed rate. It takes the same body as `set`
    and returns as soon as the target is recorded, not when the jaws arrive: a per-side worker drives
    the same force-limited stroke machine toward whatever the latest target is. Rapid updates coalesce
    (newest wins, one continuous motion), and a target that crosses the current jaw position re-aims or
    reverses the stroke in flight rather than queueing behind it. Same force limit, grasp detection,
    hold preload, coil guard and soft-kill gate as `set`. Read `/v1/gripper/{side}/state` for
    `target_closedness`, `tracking` and the last stroke outcome. Use `set` when the caller wants the
    stroke's result.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripperTarget): A continuous gripper target.

            Exactly one of `closedness` and `jaw_rad` is given. `closedness` is the hand-protocol
            convention shared with dx-manipulator's wire units: `0.0` is the commanded
            open ceiling, `1.0` the closed stop, linear in motor radians between them.
            `jaw_rad` addresses the motor directly, in the same radians the stroke
            report's `jaw_rad` uses. The backend resolves either to a motor target and
            refuses values outside the mechanism's range. The flattened
            [`GripRequest`] fields (`grip`, `grip_preload_rad`) say how hard to hold
            whatever the stroke meets on the way.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperTargetResponse200
    """

    return sync_detailed(
        side=side,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripperTarget,
) -> Response[ErrorEnvelope | GripperTargetResponse200]:
    """Post the newest streamed target for one gripper

     The non-blocking lane, for controllers that command at a fixed rate. It takes the same body as `set`
    and returns as soon as the target is recorded, not when the jaws arrive: a per-side worker drives
    the same force-limited stroke machine toward whatever the latest target is. Rapid updates coalesce
    (newest wins, one continuous motion), and a target that crosses the current jaw position re-aims or
    reverses the stroke in flight rather than queueing behind it. Same force limit, grasp detection,
    hold preload, coil guard and soft-kill gate as `set`. Read `/v1/gripper/{side}/state` for
    `target_closedness`, `tracking` and the last stroke outcome. Use `set` when the caller wants the
    stroke's result.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripperTarget): A continuous gripper target.

            Exactly one of `closedness` and `jaw_rad` is given. `closedness` is the hand-protocol
            convention shared with dx-manipulator's wire units: `0.0` is the commanded
            open ceiling, `1.0` the closed stop, linear in motor radians between them.
            `jaw_rad` addresses the motor directly, in the same radians the stroke
            report's `jaw_rad` uses. The backend resolves either to a motor target and
            refuses values outside the mechanism's range. The flattened
            [`GripRequest`] fields (`grip`, `grip_preload_rad`) say how hard to hold
            whatever the stroke meets on the way.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperTargetResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripperTarget,
) -> ErrorEnvelope | GripperTargetResponse200 | None:
    """Post the newest streamed target for one gripper

     The non-blocking lane, for controllers that command at a fixed rate. It takes the same body as `set`
    and returns as soon as the target is recorded, not when the jaws arrive: a per-side worker drives
    the same force-limited stroke machine toward whatever the latest target is. Rapid updates coalesce
    (newest wins, one continuous motion), and a target that crosses the current jaw position re-aims or
    reverses the stroke in flight rather than queueing behind it. Same force limit, grasp detection,
    hold preload, coil guard and soft-kill gate as `set`. Read `/v1/gripper/{side}/state` for
    `target_closedness`, `tracking` and the last stroke outcome. Use `set` when the caller wants the
    stroke's result.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripperTarget): A continuous gripper target.

            Exactly one of `closedness` and `jaw_rad` is given. `closedness` is the hand-protocol
            convention shared with dx-manipulator's wire units: `0.0` is the commanded
            open ceiling, `1.0` the closed stop, linear in motor radians between them.
            `jaw_rad` addresses the motor directly, in the same radians the stroke
            report's `jaw_rad` uses. The backend resolves either to a motor target and
            refuses values outside the mechanism's range. The flattened
            [`GripRequest`] fields (`grip`, `grip_preload_rad`) say how hard to hold
            whatever the stroke meets on the way.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperTargetResponse200
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
