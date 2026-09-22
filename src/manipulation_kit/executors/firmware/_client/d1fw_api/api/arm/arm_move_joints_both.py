from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_move_joints_both_body import ArmMoveJointsBothBody
from ...models.arm_move_joints_both_response_200 import ArmMoveJointsBothResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ArmMoveJointsBothBody,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/move_joints_both",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmMoveJointsBothResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmMoveJointsBothResponse200.from_dict(response.json())

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
) -> Response[ArmMoveJointsBothResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBothBody,
) -> Response[ArmMoveJointsBothResponse200 | ErrorEnvelope]:
    """Command both arms to joint targets in one guarded call

     Guarded as one dual-arm pose, so it accepts poses that two single-arm calls in sequence would not.
    Angles are degrees.

    Both arms have to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If
    either is not, the call is refused at once with `kind: refused` and no command is written to either
    arm; an arm that is merely idle carries an `advisory` in the failure envelope naming the recover
    call that energises it.

    Args:
        body (ArmMoveJointsBothBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsBothResponse200 | ErrorEnvelope]
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
    body: ArmMoveJointsBothBody,
) -> ArmMoveJointsBothResponse200 | ErrorEnvelope | None:
    """Command both arms to joint targets in one guarded call

     Guarded as one dual-arm pose, so it accepts poses that two single-arm calls in sequence would not.
    Angles are degrees.

    Both arms have to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If
    either is not, the call is refused at once with `kind: refused` and no command is written to either
    arm; an arm that is merely idle carries an `advisory` in the failure envelope naming the recover
    call that energises it.

    Args:
        body (ArmMoveJointsBothBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsBothResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBothBody,
) -> Response[ArmMoveJointsBothResponse200 | ErrorEnvelope]:
    """Command both arms to joint targets in one guarded call

     Guarded as one dual-arm pose, so it accepts poses that two single-arm calls in sequence would not.
    Angles are degrees.

    Both arms have to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If
    either is not, the call is refused at once with `kind: refused` and no command is written to either
    arm; an arm that is merely idle carries an `advisory` in the failure envelope naming the recover
    call that energises it.

    Args:
        body (ArmMoveJointsBothBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsBothResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBothBody,
) -> ArmMoveJointsBothResponse200 | ErrorEnvelope | None:
    """Command both arms to joint targets in one guarded call

     Guarded as one dual-arm pose, so it accepts poses that two single-arm calls in sequence would not.
    Angles are degrees.

    Both arms have to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If
    either is not, the call is refused at once with `kind: refused` and no command is written to either
    arm; an arm that is merely idle carries an `advisory` in the failure envelope naming the recover
    call that energises it.

    Args:
        body (ArmMoveJointsBothBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsBothResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
